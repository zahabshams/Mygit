
Linux Cross Reference
Free Electrons
Embedded Linux Experts

 • source navigation  • diff markup  • identifier search  • freetext search  • 

Version:  2.6.32 2.6.33 2.6.34 2.6.35 2.6.36 2.6.37 2.6.38 2.6.39 3.0 3.1 3.2 3.3 3.4 3.5 3.6 3.7 3.8 3.9 3.10 3.11

Architecture:  x86 arm avr32 blackfin m68k m68knommu microblaze mips powerpc sh
Linux/drivers/net/usb/cdc_ncm.c

  1 /*
  2  * cdc_ncm.c
  3  *
  4  * Copyright (C) ST-Ericsson 2010-2012
  5  * Contact: Alexey Orishko <alexey.orishko@stericsson.com>
  6  * Original author: Hans Petter Selasky <hans.petter.selasky@stericsson.com>
  7  *
  8  * USB Host Driver for Network Control Model (NCM)
  9  * http://www.usb.org/developers/devclass_docs/NCM10.zip
 10  *
 11  * The NCM encoding, decoding and initialization logic
 12  * derives from FreeBSD 8.x. if_cdce.c and if_cdcereg.h
 13  *
 14  * This software is available to you under a choice of one of two
 15  * licenses. You may choose this file to be licensed under the terms
 16  * of the GNU General Public License (GPL) Version 2 or the 2-clause
 17  * BSD license listed below:
 18  *
 19  * Redistribution and use in source and binary forms, with or without
 20  * modification, are permitted provided that the following conditions
 21  * are met:
 22  * 1. Redistributions of source code must retain the above copyright
 23  *    notice, this list of conditions and the following disclaimer.
 24  * 2. Redistributions in binary form must reproduce the above copyright
 25  *    notice, this list of conditions and the following disclaimer in the
 26  *    documentation and/or other materials provided with the distribution.
 27  *
 28  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 29  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 30  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 31  * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 32  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 33  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 34  * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 35  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 36  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 37  * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 38  * SUCH DAMAGE.
 39  */
 40 
 41 #include <linux/module.h>
 42 #include <linux/init.h>
 43 #include <linux/netdevice.h>
 44 #include <linux/ctype.h>
 45 #include <linux/ethtool.h>
 46 #include <linux/workqueue.h>
 47 #include <linux/mii.h>
 48 #include <linux/crc32.h>
 49 #include <linux/usb.h>
 50 #include <linux/hrtimer.h>
 51 #include <linux/atomic.h>
 52 #include <linux/usb/usbnet.h>
 53 #include <linux/usb/cdc.h>
 54 #include <linux/usb/cdc_ncm.h>
 55 
 56 #define DRIVER_VERSION                          "14-Mar-2012"
 57 
 58 #if IS_ENABLED(CONFIG_USB_NET_CDC_MBIM)
 59 static bool prefer_mbim = true;
 60 #else
 61 static bool prefer_mbim;
 62 #endif
 63 module_param(prefer_mbim, bool, S_IRUGO | S_IWUSR);
 64 MODULE_PARM_DESC(prefer_mbim, "Prefer MBIM setting on dual NCM/MBIM functions");
 65 
 66 static void cdc_ncm_txpath_bh(unsigned long param);
 67 static void cdc_ncm_tx_timeout_start(struct cdc_ncm_ctx *ctx);
 68 static enum hrtimer_restart cdc_ncm_tx_timer_cb(struct hrtimer *hr_timer);
 69 static struct usb_driver cdc_ncm_driver;
 70 
 71 static void
 72 cdc_ncm_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
 73 {
 74         struct usbnet *dev = netdev_priv(net);
 75 
 76         strlcpy(info->driver, dev->driver_name, sizeof(info->driver));
 77         strlcpy(info->version, DRIVER_VERSION, sizeof(info->version));
 78         strlcpy(info->fw_version, dev->driver_info->description,
 79                 sizeof(info->fw_version));
 80         usb_make_path(dev->udev, info->bus_info, sizeof(info->bus_info));
 81 }
 82 
 83 static u8 cdc_ncm_setup(struct cdc_ncm_ctx *ctx)
 84 {
 85         u32 val;
 86         u8 flags;
 87         u8 iface_no;
 88         int err;
 89         int eth_hlen;
 90         u16 ntb_fmt_supported;
 91         u32 min_dgram_size;
 92         u32 min_hdr_size;
 93         struct usbnet *dev = netdev_priv(ctx->netdev);
 94 
 95         iface_no = ctx->control->cur_altsetting->desc.bInterfaceNumber;
 96 
 97         err = usbnet_read_cmd(dev, USB_CDC_GET_NTB_PARAMETERS,
 98                               USB_TYPE_CLASS | USB_DIR_IN
 99                               |USB_RECIP_INTERFACE,
100                               0, iface_no, &ctx->ncm_parm,
101                               sizeof(ctx->ncm_parm));
102         if (err < 0) {
103                 pr_debug("failed GET_NTB_PARAMETERS\n");
104                 return 1;
105         }
106 
107         /* read correct set of parameters according to device mode */
108         ctx->rx_max = le32_to_cpu(ctx->ncm_parm.dwNtbInMaxSize);
109         ctx->tx_max = le32_to_cpu(ctx->ncm_parm.dwNtbOutMaxSize);
110         ctx->tx_remainder = le16_to_cpu(ctx->ncm_parm.wNdpOutPayloadRemainder);
111         ctx->tx_modulus = le16_to_cpu(ctx->ncm_parm.wNdpOutDivisor);
112         ctx->tx_ndp_modulus = le16_to_cpu(ctx->ncm_parm.wNdpOutAlignment);
113         /* devices prior to NCM Errata shall set this field to zero */
114         ctx->tx_max_datagrams = le16_to_cpu(ctx->ncm_parm.wNtbOutMaxDatagrams);
115         ntb_fmt_supported = le16_to_cpu(ctx->ncm_parm.bmNtbFormatsSupported);
116 
117         eth_hlen = ETH_HLEN;
118         min_dgram_size = CDC_NCM_MIN_DATAGRAM_SIZE;
119         min_hdr_size = CDC_NCM_MIN_HDR_SIZE;
120         if (ctx->mbim_desc != NULL) {
121                 flags = ctx->mbim_desc->bmNetworkCapabilities;
122                 eth_hlen = 0;
123                 min_dgram_size = CDC_MBIM_MIN_DATAGRAM_SIZE;
124                 min_hdr_size = 0;
125         } else if (ctx->func_desc != NULL) {
126                 flags = ctx->func_desc->bmNetworkCapabilities;
127         } else {
128                 flags = 0;
129         }
130 
131         pr_debug("dwNtbInMaxSize=%u dwNtbOutMaxSize=%u "
132                  "wNdpOutPayloadRemainder=%u wNdpOutDivisor=%u "
133                  "wNdpOutAlignment=%u wNtbOutMaxDatagrams=%u flags=0x%x\n",
134                  ctx->rx_max, ctx->tx_max, ctx->tx_remainder, ctx->tx_modulus,
135                  ctx->tx_ndp_modulus, ctx->tx_max_datagrams, flags);
136 
137         /* max count of tx datagrams */
138         if ((ctx->tx_max_datagrams == 0) ||
139                         (ctx->tx_max_datagrams > CDC_NCM_DPT_DATAGRAMS_MAX))
140                 ctx->tx_max_datagrams = CDC_NCM_DPT_DATAGRAMS_MAX;
141 
142         /* verify maximum size of received NTB in bytes */
143         if (ctx->rx_max < USB_CDC_NCM_NTB_MIN_IN_SIZE) {
144                 pr_debug("Using min receive length=%d\n",
145                                                 USB_CDC_NCM_NTB_MIN_IN_SIZE);
146                 ctx->rx_max = USB_CDC_NCM_NTB_MIN_IN_SIZE;
147         }
148 
149         if (ctx->rx_max > CDC_NCM_NTB_MAX_SIZE_RX) {
150                 pr_debug("Using default maximum receive length=%d\n",
151                                                 CDC_NCM_NTB_MAX_SIZE_RX);
152                 ctx->rx_max = CDC_NCM_NTB_MAX_SIZE_RX;
153         }
154 
155         /* inform device about NTB input size changes */
156         if (ctx->rx_max != le32_to_cpu(ctx->ncm_parm.dwNtbInMaxSize)) {
157                 __le32 dwNtbInMaxSize = cpu_to_le32(ctx->rx_max);
158 
159                 err = usbnet_write_cmd(dev, USB_CDC_SET_NTB_INPUT_SIZE,
160                                        USB_TYPE_CLASS | USB_DIR_OUT
161                                        | USB_RECIP_INTERFACE,
162                                        0, iface_no, &dwNtbInMaxSize, 4);
163                 if (err < 0)
164                         pr_debug("Setting NTB Input Size failed\n");
165         }
166 
167         /* verify maximum size of transmitted NTB in bytes */
168         if ((ctx->tx_max <
169             (min_hdr_size + min_dgram_size)) ||
170             (ctx->tx_max > CDC_NCM_NTB_MAX_SIZE_TX)) {
171                 pr_debug("Using default maximum transmit length=%d\n",
172                                                 CDC_NCM_NTB_MAX_SIZE_TX);
173                 ctx->tx_max = CDC_NCM_NTB_MAX_SIZE_TX;
174         }
175 
176         /*
177          * verify that the structure alignment is:
178          * - power of two
179          * - not greater than the maximum transmit length
180          * - not less than four bytes
181          */
182         val = ctx->tx_ndp_modulus;
183 
184         if ((val < USB_CDC_NCM_NDP_ALIGN_MIN_SIZE) ||
185             (val != ((-val) & val)) || (val >= ctx->tx_max)) {
186                 pr_debug("Using default alignment: 4 bytes\n");
187                 ctx->tx_ndp_modulus = USB_CDC_NCM_NDP_ALIGN_MIN_SIZE;
188         }
189 
190         /*
191          * verify that the payload alignment is:
192          * - power of two
193          * - not greater than the maximum transmit length
194          * - not less than four bytes
195          */
196         val = ctx->tx_modulus;
197 
198         if ((val < USB_CDC_NCM_NDP_ALIGN_MIN_SIZE) ||
199             (val != ((-val) & val)) || (val >= ctx->tx_max)) {
200                 pr_debug("Using default transmit modulus: 4 bytes\n");
201                 ctx->tx_modulus = USB_CDC_NCM_NDP_ALIGN_MIN_SIZE;
202         }
203 
204         /* verify the payload remainder */
205         if (ctx->tx_remainder >= ctx->tx_modulus) {
206                 pr_debug("Using default transmit remainder: 0 bytes\n");
207                 ctx->tx_remainder = 0;
208         }
209 
210         /* adjust TX-remainder according to NCM specification. */
211         ctx->tx_remainder = ((ctx->tx_remainder - eth_hlen) &
212                              (ctx->tx_modulus - 1));
213 
214         /* additional configuration */
215 
216         /* set CRC Mode */
217         if (flags & USB_CDC_NCM_NCAP_CRC_MODE) {
218                 err = usbnet_write_cmd(dev, USB_CDC_SET_CRC_MODE,
219                                        USB_TYPE_CLASS | USB_DIR_OUT
220                                        | USB_RECIP_INTERFACE,
221                                        USB_CDC_NCM_CRC_NOT_APPENDED,
222                                        iface_no, NULL, 0);
223                 if (err < 0)
224                         pr_debug("Setting CRC mode off failed\n");
225         }
226 
227         /* set NTB format, if both formats are supported */
228         if (ntb_fmt_supported & USB_CDC_NCM_NTH32_SIGN) {
229                 err = usbnet_write_cmd(dev, USB_CDC_SET_NTB_FORMAT,
230                                        USB_TYPE_CLASS | USB_DIR_OUT
231                                        | USB_RECIP_INTERFACE,
232                                        USB_CDC_NCM_NTB16_FORMAT,
233                                        iface_no, NULL, 0);
234                 if (err < 0)
235                         pr_debug("Setting NTB format to 16-bit failed\n");
236         }
237 
238         ctx->max_datagram_size = min_dgram_size;
239 
240         /* set Max Datagram Size (MTU) */
241         if (flags & USB_CDC_NCM_NCAP_MAX_DATAGRAM_SIZE) {
242                 __le16 max_datagram_size;
243                 u16 eth_max_sz;
244                 if (ctx->ether_desc != NULL)
245                         eth_max_sz = le16_to_cpu(ctx->ether_desc->wMaxSegmentSize);
246                 else if (ctx->mbim_desc != NULL)
247                         eth_max_sz = le16_to_cpu(ctx->mbim_desc->wMaxSegmentSize);
248                 else
249                         goto max_dgram_err;
250 
251                 err = usbnet_read_cmd(dev, USB_CDC_GET_MAX_DATAGRAM_SIZE,
252                                       USB_TYPE_CLASS | USB_DIR_IN
253                                       | USB_RECIP_INTERFACE,
254                                       0, iface_no, &max_datagram_size, 2);
255                 if (err < 0) {
256                         pr_debug("GET_MAX_DATAGRAM_SIZE failed, use size=%u\n",
257                                  min_dgram_size);
258                 } else {
259                         ctx->max_datagram_size =
260                                 le16_to_cpu(max_datagram_size);
261                         /* Check Eth descriptor value */
262                         if (ctx->max_datagram_size > eth_max_sz)
263                                         ctx->max_datagram_size = eth_max_sz;
264 
265                         if (ctx->max_datagram_size > CDC_NCM_MAX_DATAGRAM_SIZE)
266                                 ctx->max_datagram_size = CDC_NCM_MAX_DATAGRAM_SIZE;
267 
268                         if (ctx->max_datagram_size < min_dgram_size)
269                                 ctx->max_datagram_size = min_dgram_size;
270 
271                         /* if value changed, update device */
272                         if (ctx->max_datagram_size !=
273                                         le16_to_cpu(max_datagram_size)) {
274                                 err = usbnet_write_cmd(dev,
275                                                 USB_CDC_SET_MAX_DATAGRAM_SIZE,
276                                                 USB_TYPE_CLASS | USB_DIR_OUT
277                                                  | USB_RECIP_INTERFACE,
278                                                 0,
279                                                 iface_no, &max_datagram_size,
280                                                 2);
281                                 if (err < 0)
282                                         pr_debug("SET_MAX_DGRAM_SIZE failed\n");
283                         }
284                 }
285         }
286 
287 max_dgram_err:
288         if (ctx->netdev->mtu != (ctx->max_datagram_size - eth_hlen))
289                 ctx->netdev->mtu = ctx->max_datagram_size - eth_hlen;
290 
291         return 0;
292 }
293 
294 static void
295 cdc_ncm_find_endpoints(struct cdc_ncm_ctx *ctx, struct usb_interface *intf)
296 {
297         struct usb_host_endpoint *e;
298         u8 ep;
299 
300         for (ep = 0; ep < intf->cur_altsetting->desc.bNumEndpoints; ep++) {
301 
302                 e = intf->cur_altsetting->endpoint + ep;
303                 switch (e->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
304                 case USB_ENDPOINT_XFER_INT:
305                         if (usb_endpoint_dir_in(&e->desc)) {
306                                 if (ctx->status_ep == NULL)
307                                         ctx->status_ep = e;
308                         }
309                         break;
310 
311                 case USB_ENDPOINT_XFER_BULK:
312                         if (usb_endpoint_dir_in(&e->desc)) {
313                                 if (ctx->in_ep == NULL)
314                                         ctx->in_ep = e;
315                         } else {
316                                 if (ctx->out_ep == NULL)
317                                         ctx->out_ep = e;
318                         }
319                         break;
320 
321                 default:
322                         break;
323                 }
324         }
325 }
326 
327 static void cdc_ncm_free(struct cdc_ncm_ctx *ctx)
328 {
329         if (ctx == NULL)
330                 return;
331 
332         if (ctx->tx_rem_skb != NULL) {
333                 dev_kfree_skb_any(ctx->tx_rem_skb);
334                 ctx->tx_rem_skb = NULL;
335         }
336 
337         if (ctx->tx_curr_skb != NULL) {
338                 dev_kfree_skb_any(ctx->tx_curr_skb);
339                 ctx->tx_curr_skb = NULL;
340         }
341 
342         kfree(ctx);
343 }
344 
345 static const struct ethtool_ops cdc_ncm_ethtool_ops = {
346         .get_drvinfo = cdc_ncm_get_drvinfo,
347         .get_link = usbnet_get_link,
348         .get_msglevel = usbnet_get_msglevel,
349         .set_msglevel = usbnet_set_msglevel,
350         .get_settings = usbnet_get_settings,
351         .set_settings = usbnet_set_settings,
352         .nway_reset = usbnet_nway_reset,
353 };
354 
355 int cdc_ncm_bind_common(struct usbnet *dev, struct usb_interface *intf, u8 data_altsetting)
356 {
357         struct cdc_ncm_ctx *ctx;
358         struct usb_driver *driver;
359         u8 *buf;
360         int len;
361         int temp;
362         u8 iface_no;
363 
364         ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
365         if (!ctx)
366                 return -ENOMEM;
367 
368         hrtimer_init(&ctx->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
369         ctx->tx_timer.function = &cdc_ncm_tx_timer_cb;
370         ctx->bh.data = (unsigned long)ctx;
371         ctx->bh.func = cdc_ncm_txpath_bh;
372         atomic_set(&ctx->stop, 0);
373         spin_lock_init(&ctx->mtx);
374         ctx->netdev = dev->net;
375 
376         /* store ctx pointer in device data field */
377         dev->data[0] = (unsigned long)ctx;
378 
379         /* get some pointers */
380         driver = driver_of(intf);
381         buf = intf->cur_altsetting->extra;
382         len = intf->cur_altsetting->extralen;
383 
384         ctx->udev = dev->udev;
385         ctx->intf = intf;
386 
387         /* parse through descriptors associated with control interface */
388         while ((len > 0) && (buf[0] > 2) && (buf[0] <= len)) {
389 
390                 if (buf[1] != USB_DT_CS_INTERFACE)
391                         goto advance;
392 
393                 switch (buf[2]) {
394                 case USB_CDC_UNION_TYPE:
395                         if (buf[0] < sizeof(*(ctx->union_desc)))
396                                 break;
397 
398                         ctx->union_desc =
399                                         (const struct usb_cdc_union_desc *)buf;
400 
401                         ctx->control = usb_ifnum_to_if(dev->udev,
402                                         ctx->union_desc->bMasterInterface0);
403                         ctx->data = usb_ifnum_to_if(dev->udev,
404                                         ctx->union_desc->bSlaveInterface0);
405                         break;
406 
407                 case USB_CDC_ETHERNET_TYPE:
408                         if (buf[0] < sizeof(*(ctx->ether_desc)))
409                                 break;
410 
411                         ctx->ether_desc =
412                                         (const struct usb_cdc_ether_desc *)buf;
413                         dev->hard_mtu =
414                                 le16_to_cpu(ctx->ether_desc->wMaxSegmentSize);
415 
416                         if (dev->hard_mtu < CDC_NCM_MIN_DATAGRAM_SIZE)
417                                 dev->hard_mtu = CDC_NCM_MIN_DATAGRAM_SIZE;
418                         else if (dev->hard_mtu > CDC_NCM_MAX_DATAGRAM_SIZE)
419                                 dev->hard_mtu = CDC_NCM_MAX_DATAGRAM_SIZE;
420                         break;
421 
422                 case USB_CDC_NCM_TYPE:
423                         if (buf[0] < sizeof(*(ctx->func_desc)))
424                                 break;
425 
426                         ctx->func_desc = (const struct usb_cdc_ncm_desc *)buf;
427                         break;
428 
429                 case USB_CDC_MBIM_TYPE:
430                         if (buf[0] < sizeof(*(ctx->mbim_desc)))
431                                 break;
432 
433                         ctx->mbim_desc = (const struct usb_cdc_mbim_desc *)buf;
434                         break;
435 
436                 default:
437                         break;
438                 }
439 advance:
440                 /* advance to next descriptor */
441                 temp = buf[0];
442                 buf += temp;
443                 len -= temp;
444         }
445 
446         /* some buggy devices have an IAD but no CDC Union */
447         if (!ctx->union_desc && intf->intf_assoc && intf->intf_assoc->bInterfaceCount == 2) {
448                 ctx->control = intf;
449                 ctx->data = usb_ifnum_to_if(dev->udev, intf->cur_altsetting->desc.bInterfaceNumber + 1);
450                 dev_dbg(&intf->dev, "CDC Union missing - got slave from IAD\n");
451         }
452 
453         /* check if we got everything */
454         if ((ctx->control == NULL) || (ctx->data == NULL) ||
455             ((!ctx->mbim_desc) && ((ctx->ether_desc == NULL) || (ctx->control != intf))))
456                 goto error;
457 
458         /* claim data interface, if different from control */
459         if (ctx->data != ctx->control) {
460                 temp = usb_driver_claim_interface(driver, ctx->data, dev);
461                 if (temp)
462                         goto error;
463         }
464 
465         iface_no = ctx->data->cur_altsetting->desc.bInterfaceNumber;
466 
467         /* reset data interface */
468         temp = usb_set_interface(dev->udev, iface_no, 0);
469         if (temp)
470                 goto error2;
471 
472         /* initialize data interface */
473         if (cdc_ncm_setup(ctx))
474                 goto error2;
475 
476         /* configure data interface */
477         temp = usb_set_interface(dev->udev, iface_no, data_altsetting);
478         if (temp)
479                 goto error2;
480 
481         cdc_ncm_find_endpoints(ctx, ctx->data);
482         cdc_ncm_find_endpoints(ctx, ctx->control);
483 
484         if ((ctx->in_ep == NULL) || (ctx->out_ep == NULL) ||
485             (ctx->status_ep == NULL))
486                 goto error2;
487 
488         dev->net->ethtool_ops = &cdc_ncm_ethtool_ops;
489 
490         usb_set_intfdata(ctx->data, dev);
491         usb_set_intfdata(ctx->control, dev);
492         usb_set_intfdata(ctx->intf, dev);
493 
494         if (ctx->ether_desc) {
495                 temp = usbnet_get_ethernet_addr(dev, ctx->ether_desc->iMACAddress);
496                 if (temp)
497                         goto error2;
498                 dev_info(&dev->udev->dev, "MAC-Address: %pM\n", dev->net->dev_addr);
499         }
500 
501 
502         dev->in = usb_rcvbulkpipe(dev->udev,
503                 ctx->in_ep->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
504         dev->out = usb_sndbulkpipe(dev->udev,
505                 ctx->out_ep->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
506         dev->status = ctx->status_ep;
507         dev->rx_urb_size = ctx->rx_max;
508 
509         ctx->tx_speed = ctx->rx_speed = 0;
510         return 0;
511 
512 error2:
513         usb_set_intfdata(ctx->control, NULL);
514         usb_set_intfdata(ctx->data, NULL);
515         if (ctx->data != ctx->control)
516                 usb_driver_release_interface(driver, ctx->data);
517 error:
518         cdc_ncm_free((struct cdc_ncm_ctx *)dev->data[0]);
519         dev->data[0] = 0;
520         dev_info(&dev->udev->dev, "bind() failure\n");
521         return -ENODEV;
522 }
523 EXPORT_SYMBOL_GPL(cdc_ncm_bind_common);
524 
525 void cdc_ncm_unbind(struct usbnet *dev, struct usb_interface *intf)
526 {
527         struct cdc_ncm_ctx *ctx = (struct cdc_ncm_ctx *)dev->data[0];
528         struct usb_driver *driver = driver_of(intf);
529 
530         if (ctx == NULL)
531                 return;         /* no setup */
532 
533         atomic_set(&ctx->stop, 1);
534 
535         if (hrtimer_active(&ctx->tx_timer))
536                 hrtimer_cancel(&ctx->tx_timer);
537 
538         tasklet_kill(&ctx->bh);
539 
540         /* handle devices with combined control and data interface */
541         if (ctx->control == ctx->data)
542                 ctx->data = NULL;
543 
544         /* disconnect master --> disconnect slave */
545         if (intf == ctx->control && ctx->data) {
546                 usb_set_intfdata(ctx->data, NULL);
547                 usb_driver_release_interface(driver, ctx->data);
548                 ctx->data = NULL;
549 
550         } else if (intf == ctx->data && ctx->control) {
551                 usb_set_intfdata(ctx->control, NULL);
552                 usb_driver_release_interface(driver, ctx->control);
553                 ctx->control = NULL;
554         }
555 
556         usb_set_intfdata(ctx->intf, NULL);
557         cdc_ncm_free(ctx);
558 }
559 EXPORT_SYMBOL_GPL(cdc_ncm_unbind);
560 
561 /* Select the MBIM altsetting iff it is preferred and available,
562  * returning the number of the corresponding data interface altsetting
563  */
564 u8 cdc_ncm_select_altsetting(struct usbnet *dev, struct usb_interface *intf)
565 {
566         struct usb_host_interface *alt;
567 
568         /* The MBIM spec defines a NCM compatible default altsetting,
569          * which we may have matched:
570          *
571          *  "Functions that implement both NCM 1.0 and MBIM (an
572          *   “NCM/MBIM function”) according to this recommendation
573          *   shall provide two alternate settings for the
574          *   Communication Interface.  Alternate setting 0, and the
575          *   associated class and endpoint descriptors, shall be
576          *   constructed according to the rules given for the
577          *   Communication Interface in section 5 of [USBNCM10].
578          *   Alternate setting 1, and the associated class and
579          *   endpoint descriptors, shall be constructed according to
580          *   the rules given in section 6 (USB Device Model) of this
581          *   specification."
582          */
583         if (prefer_mbim && intf->num_altsetting == 2) {
584                 alt = usb_altnum_to_altsetting(intf, CDC_NCM_COMM_ALTSETTING_MBIM);
585                 if (alt && cdc_ncm_comm_intf_is_mbim(alt) &&
586                     !usb_set_interface(dev->udev,
587                                        intf->cur_altsetting->desc.bInterfaceNumber,
588                                        CDC_NCM_COMM_ALTSETTING_MBIM))
589                         return CDC_NCM_DATA_ALTSETTING_MBIM;
590         }
591         return CDC_NCM_DATA_ALTSETTING_NCM;
592 }
593 EXPORT_SYMBOL_GPL(cdc_ncm_select_altsetting);
594 
595 static int cdc_ncm_bind(struct usbnet *dev, struct usb_interface *intf)
596 {
597         int ret;
598 
599         /* MBIM backwards compatible function? */
600         cdc_ncm_select_altsetting(dev, intf);
601         if (cdc_ncm_comm_intf_is_mbim(intf->cur_altsetting))
602                 return -ENODEV;
603 
604         /* NCM data altsetting is always 1 */
605         ret = cdc_ncm_bind_common(dev, intf, 1);
606 
607         /*
608          * We should get an event when network connection is "connected" or
609          * "disconnected". Set network connection in "disconnected" state
610          * (carrier is OFF) during attach, so the IP network stack does not
611          * start IPv6 negotiation and more.
612          */
613         usbnet_link_change(dev, 0, 0);
614         return ret;
615 }
616 
617 static void cdc_ncm_align_tail(struct sk_buff *skb, size_t modulus, size_t remainder, size_t max)
618 {
619         size_t align = ALIGN(skb->len, modulus) - skb->len + remainder;
620 
621         if (skb->len + align > max)
622                 align = max - skb->len;
623         if (align && skb_tailroom(skb) >= align)
624                 memset(skb_put(skb, align), 0, align);
625 }
626 
627 /* return a pointer to a valid struct usb_cdc_ncm_ndp16 of type sign, possibly
628  * allocating a new one within skb
629  */
630 static struct usb_cdc_ncm_ndp16 *cdc_ncm_ndp(struct cdc_ncm_ctx *ctx, struct sk_buff *skb, __le32 sign, size_t reserve)
631 {
632         struct usb_cdc_ncm_ndp16 *ndp16 = NULL;
633         struct usb_cdc_ncm_nth16 *nth16 = (void *)skb->data;
634         size_t ndpoffset = le16_to_cpu(nth16->wNdpIndex);
635 
636         /* follow the chain of NDPs, looking for a match */
637         while (ndpoffset) {
638                 ndp16 = (struct usb_cdc_ncm_ndp16 *)(skb->data + ndpoffset);
639                 if  (ndp16->dwSignature == sign)
640                         return ndp16;
641                 ndpoffset = le16_to_cpu(ndp16->wNextNdpIndex);
642         }
643 
644         /* align new NDP */
645         cdc_ncm_align_tail(skb, ctx->tx_ndp_modulus, 0, ctx->tx_max);
646 
647         /* verify that there is room for the NDP and the datagram (reserve) */
648         if ((ctx->tx_max - skb->len - reserve) < CDC_NCM_NDP_SIZE)
649                 return NULL;
650 
651         /* link to it */
652         if (ndp16)
653                 ndp16->wNextNdpIndex = cpu_to_le16(skb->len);
654         else
655                 nth16->wNdpIndex = cpu_to_le16(skb->len);
656 
657         /* push a new empty NDP */
658         ndp16 = (struct usb_cdc_ncm_ndp16 *)memset(skb_put(skb, CDC_NCM_NDP_SIZE), 0, CDC_NCM_NDP_SIZE);
659         ndp16->dwSignature = sign;
660         ndp16->wLength = cpu_to_le16(sizeof(struct usb_cdc_ncm_ndp16) + sizeof(struct usb_cdc_ncm_dpe16));
661         return ndp16;
662 }
663 
664 struct sk_buff *
665 cdc_ncm_fill_tx_frame(struct cdc_ncm_ctx *ctx, struct sk_buff *skb, __le32 sign)
666 {
667         struct usb_cdc_ncm_nth16 *nth16;
668         struct usb_cdc_ncm_ndp16 *ndp16;
669         struct sk_buff *skb_out;
670         u16 n = 0, index, ndplen;
671         u8 ready2send = 0;
672 
673         /* if there is a remaining skb, it gets priority */
674         if (skb != NULL) {
675                 swap(skb, ctx->tx_rem_skb);
676                 swap(sign, ctx->tx_rem_sign);
677         } else {
678                 ready2send = 1;
679         }
680 
681         /* check if we are resuming an OUT skb */
682         skb_out = ctx->tx_curr_skb;
683 
684         /* allocate a new OUT skb */
685         if (!skb_out) {
686                 skb_out = alloc_skb((ctx->tx_max + 1), GFP_ATOMIC);
687                 if (skb_out == NULL) {
688                         if (skb != NULL) {
689                                 dev_kfree_skb_any(skb);
690                                 ctx->netdev->stats.tx_dropped++;
691                         }
692                         goto exit_no_skb;
693                 }
694                 /* fill out the initial 16-bit NTB header */
695                 nth16 = (struct usb_cdc_ncm_nth16 *)memset(skb_put(skb_out, sizeof(struct usb_cdc_ncm_nth16)), 0, sizeof(struct usb_cdc_ncm_nth16));
696                 nth16->dwSignature = cpu_to_le32(USB_CDC_NCM_NTH16_SIGN);
697                 nth16->wHeaderLength = cpu_to_le16(sizeof(struct usb_cdc_ncm_nth16));
698                 nth16->wSequence = cpu_to_le16(ctx->tx_seq++);
699 
700                 /* count total number of frames in this NTB */
701                 ctx->tx_curr_frame_num = 0;
702         }
703 
704         for (n = ctx->tx_curr_frame_num; n < ctx->tx_max_datagrams; n++) {
705                 /* send any remaining skb first */
706                 if (skb == NULL) {
707                         skb = ctx->tx_rem_skb;
708                         sign = ctx->tx_rem_sign;
709                         ctx->tx_rem_skb = NULL;
710 
711                         /* check for end of skb */
712                         if (skb == NULL)
713                                 break;
714                 }
715 
716                 /* get the appropriate NDP for this skb */
717                 ndp16 = cdc_ncm_ndp(ctx, skb_out, sign, skb->len + ctx->tx_modulus + ctx->tx_remainder);
718 
719                 /* align beginning of next frame */
720                 cdc_ncm_align_tail(skb_out,  ctx->tx_modulus, ctx->tx_remainder, ctx->tx_max);
721 
722                 /* check if we had enough room left for both NDP and frame */
723                 if (!ndp16 || skb_out->len + skb->len > ctx->tx_max) {
724                         if (n == 0) {
725                                 /* won't fit, MTU problem? */
726                                 dev_kfree_skb_any(skb);
727                                 skb = NULL;
728                                 ctx->netdev->stats.tx_dropped++;
729                         } else {
730                                 /* no room for skb - store for later */
731                                 if (ctx->tx_rem_skb != NULL) {
732                                         dev_kfree_skb_any(ctx->tx_rem_skb);
733                                         ctx->netdev->stats.tx_dropped++;
734                                 }
735                                 ctx->tx_rem_skb = skb;
736                                 ctx->tx_rem_sign = sign;
737                                 skb = NULL;
738                                 ready2send = 1;
739                         }
740                         break;
741                 }
742 
743                 /* calculate frame number withing this NDP */
744                 ndplen = le16_to_cpu(ndp16->wLength);
745                 index = (ndplen - sizeof(struct usb_cdc_ncm_ndp16)) / sizeof(struct usb_cdc_ncm_dpe16) - 1;
746 
747                 /* OK, add this skb */
748                 ndp16->dpe16[index].wDatagramLength = cpu_to_le16(skb->len);
749                 ndp16->dpe16[index].wDatagramIndex = cpu_to_le16(skb_out->len);
750                 ndp16->wLength = cpu_to_le16(ndplen + sizeof(struct usb_cdc_ncm_dpe16));
751                 memcpy(skb_put(skb_out, skb->len), skb->data, skb->len);
752                 dev_kfree_skb_any(skb);
753                 skb = NULL;
754 
755                 /* send now if this NDP is full */
756                 if (index >= CDC_NCM_DPT_DATAGRAMS_MAX) {
757                         ready2send = 1;
758                         break;
759                 }
760         }
761 
762         /* free up any dangling skb */
763         if (skb != NULL) {
764                 dev_kfree_skb_any(skb);
765                 skb = NULL;
766                 ctx->netdev->stats.tx_dropped++;
767         }
768 
769         ctx->tx_curr_frame_num = n;
770 
771         if (n == 0) {
772                 /* wait for more frames */
773                 /* push variables */
774                 ctx->tx_curr_skb = skb_out;
775                 goto exit_no_skb;
776 
777         } else if ((n < ctx->tx_max_datagrams) && (ready2send == 0)) {
778                 /* wait for more frames */
779                 /* push variables */
780                 ctx->tx_curr_skb = skb_out;
781                 /* set the pending count */
782                 if (n < CDC_NCM_RESTART_TIMER_DATAGRAM_CNT)
783                         ctx->tx_timer_pending = CDC_NCM_TIMER_PENDING_CNT;
784                 goto exit_no_skb;
785 
786         } else {
787                 /* frame goes out */
788                 /* variables will be reset at next call */
789         }
790 
791         /*
792          * If collected data size is less or equal CDC_NCM_MIN_TX_PKT bytes,
793          * we send buffers as it is. If we get more data, it would be more
794          * efficient for USB HS mobile device with DMA engine to receive a full
795          * size NTB, than canceling DMA transfer and receiving a short packet.
796          */
797         if (skb_out->len > CDC_NCM_MIN_TX_PKT)
798                 /* final zero padding */
799                 memset(skb_put(skb_out, ctx->tx_max - skb_out->len), 0, ctx->tx_max - skb_out->len);
800 
801         /* do we need to prevent a ZLP? */
802         if (((skb_out->len % le16_to_cpu(ctx->out_ep->desc.wMaxPacketSize)) == 0) &&
803             (skb_out->len < le32_to_cpu(ctx->ncm_parm.dwNtbOutMaxSize)) && skb_tailroom(skb_out))
804                 *skb_put(skb_out, 1) = 0;       /* force short packet */
805 
806         /* set final frame length */
807         nth16 = (struct usb_cdc_ncm_nth16 *)skb_out->data;
808         nth16->wBlockLength = cpu_to_le16(skb_out->len);
809 
810         /* return skb */
811         ctx->tx_curr_skb = NULL;
812         ctx->netdev->stats.tx_packets += ctx->tx_curr_frame_num;
813         return skb_out;
814 
815 exit_no_skb:
816         /* Start timer, if there is a remaining skb */
817         if (ctx->tx_curr_skb != NULL)
818                 cdc_ncm_tx_timeout_start(ctx);
819         return NULL;
820 }
821 EXPORT_SYMBOL_GPL(cdc_ncm_fill_tx_frame);
822 
823 static void cdc_ncm_tx_timeout_start(struct cdc_ncm_ctx *ctx)
824 {
825         /* start timer, if not already started */
826         if (!(hrtimer_active(&ctx->tx_timer) || atomic_read(&ctx->stop)))
827                 hrtimer_start(&ctx->tx_timer,
828                                 ktime_set(0, CDC_NCM_TIMER_INTERVAL),
829                                 HRTIMER_MODE_REL);
830 }
831 
832 static enum hrtimer_restart cdc_ncm_tx_timer_cb(struct hrtimer *timer)
833 {
834         struct cdc_ncm_ctx *ctx =
835                         container_of(timer, struct cdc_ncm_ctx, tx_timer);
836 
837         if (!atomic_read(&ctx->stop))
838                 tasklet_schedule(&ctx->bh);
839         return HRTIMER_NORESTART;
840 }
841 
842 static void cdc_ncm_txpath_bh(unsigned long param)
843 {
844         struct cdc_ncm_ctx *ctx = (struct cdc_ncm_ctx *)param;
845 
846         spin_lock_bh(&ctx->mtx);
847         if (ctx->tx_timer_pending != 0) {
848                 ctx->tx_timer_pending--;
849                 cdc_ncm_tx_timeout_start(ctx);
850                 spin_unlock_bh(&ctx->mtx);
851         } else if (ctx->netdev != NULL) {
852                 spin_unlock_bh(&ctx->mtx);
853                 netif_tx_lock_bh(ctx->netdev);
854                 usbnet_start_xmit(NULL, ctx->netdev);
855                 netif_tx_unlock_bh(ctx->netdev);
856         } else {
857                 spin_unlock_bh(&ctx->mtx);
858         }
859 }
860 
861 static struct sk_buff *
862 cdc_ncm_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
863 {
864         struct sk_buff *skb_out;
865         struct cdc_ncm_ctx *ctx = (struct cdc_ncm_ctx *)dev->data[0];
866 
867         /*
868          * The Ethernet API we are using does not support transmitting
869          * multiple Ethernet frames in a single call. This driver will
870          * accumulate multiple Ethernet frames and send out a larger
871          * USB frame when the USB buffer is full or when a single jiffies
872          * timeout happens.
873          */
874         if (ctx == NULL)
875                 goto error;
876 
877         spin_lock_bh(&ctx->mtx);
878         skb_out = cdc_ncm_fill_tx_frame(ctx, skb, cpu_to_le32(USB_CDC_NCM_NDP16_NOCRC_SIGN));
879         spin_unlock_bh(&ctx->mtx);
880         return skb_out;
881 
882 error:
883         if (skb != NULL)
884                 dev_kfree_skb_any(skb);
885 
886         return NULL;
887 }
888 
889 /* verify NTB header and return offset of first NDP, or negative error */
890 int cdc_ncm_rx_verify_nth16(struct cdc_ncm_ctx *ctx, struct sk_buff *skb_in)
891 {
892         struct usb_cdc_ncm_nth16 *nth16;
893         int len;
894         int ret = -EINVAL;
895 
896         if (ctx == NULL)
897                 goto error;
898 
899         if (skb_in->len < (sizeof(struct usb_cdc_ncm_nth16) +
900                                         sizeof(struct usb_cdc_ncm_ndp16))) {
901                 pr_debug("frame too short\n");
902                 goto error;
903         }
904 
905         nth16 = (struct usb_cdc_ncm_nth16 *)skb_in->data;
906 
907         if (le32_to_cpu(nth16->dwSignature) != USB_CDC_NCM_NTH16_SIGN) {
908                 pr_debug("invalid NTH16 signature <%u>\n",
909                                         le32_to_cpu(nth16->dwSignature));
910                 goto error;
911         }
912 
913         len = le16_to_cpu(nth16->wBlockLength);
914         if (len > ctx->rx_max) {
915                 pr_debug("unsupported NTB block length %u/%u\n", len,
916                                                                 ctx->rx_max);
917                 goto error;
918         }
919 
920         if ((ctx->rx_seq + 1) != le16_to_cpu(nth16->wSequence) &&
921                 (ctx->rx_seq || le16_to_cpu(nth16->wSequence)) &&
922                 !((ctx->rx_seq == 0xffff) && !le16_to_cpu(nth16->wSequence))) {
923                 pr_debug("sequence number glitch prev=%d curr=%d\n",
924                                 ctx->rx_seq, le16_to_cpu(nth16->wSequence));
925         }
926         ctx->rx_seq = le16_to_cpu(nth16->wSequence);
927 
928         ret = le16_to_cpu(nth16->wNdpIndex);
929 error:
930         return ret;
931 }
932 EXPORT_SYMBOL_GPL(cdc_ncm_rx_verify_nth16);
933 
934 /* verify NDP header and return number of datagrams, or negative error */
935 int cdc_ncm_rx_verify_ndp16(struct sk_buff *skb_in, int ndpoffset)
936 {
937         struct usb_cdc_ncm_ndp16 *ndp16;
938         int ret = -EINVAL;
939 
940         if ((ndpoffset + sizeof(struct usb_cdc_ncm_ndp16)) > skb_in->len) {
941                 pr_debug("invalid NDP offset  <%u>\n", ndpoffset);
942                 goto error;
943         }
944         ndp16 = (struct usb_cdc_ncm_ndp16 *)(skb_in->data + ndpoffset);
945 
946         if (le16_to_cpu(ndp16->wLength) < USB_CDC_NCM_NDP16_LENGTH_MIN) {
947                 pr_debug("invalid DPT16 length <%u>\n",
948                                         le32_to_cpu(ndp16->dwSignature));
949                 goto error;
950         }
951 
952         ret = ((le16_to_cpu(ndp16->wLength) -
953                                         sizeof(struct usb_cdc_ncm_ndp16)) /
954                                         sizeof(struct usb_cdc_ncm_dpe16));
955         ret--; /* we process NDP entries except for the last one */
956 
957         if ((sizeof(struct usb_cdc_ncm_ndp16) + ret * (sizeof(struct usb_cdc_ncm_dpe16))) >
958                                                                 skb_in->len) {
959                 pr_debug("Invalid nframes = %d\n", ret);
960                 ret = -EINVAL;
961         }
962 
963 error:
964         return ret;
965 }
966 EXPORT_SYMBOL_GPL(cdc_ncm_rx_verify_ndp16);
967 
968 static int cdc_ncm_rx_fixup(struct usbnet *dev, struct sk_buff *skb_in)
969 {
970         struct sk_buff *skb;
971         struct cdc_ncm_ctx *ctx = (struct cdc_ncm_ctx *)dev->data[0];
972         int len;
973         int nframes;
974         int x;
975         int offset;
976         struct usb_cdc_ncm_ndp16 *ndp16;
977         struct usb_cdc_ncm_dpe16 *dpe16;
978         int ndpoffset;
979         int loopcount = 50; /* arbitrary max preventing infinite loop */
980 
981         ndpoffset = cdc_ncm_rx_verify_nth16(ctx, skb_in);
982         if (ndpoffset < 0)
983                 goto error;
984 
985 next_ndp:
986         nframes = cdc_ncm_rx_verify_ndp16(skb_in, ndpoffset);
987         if (nframes < 0)
988                 goto error;
989 
990         ndp16 = (struct usb_cdc_ncm_ndp16 *)(skb_in->data + ndpoffset);
991 
992         if (le32_to_cpu(ndp16->dwSignature) != USB_CDC_NCM_NDP16_NOCRC_SIGN) {
993                 pr_debug("invalid DPT16 signature <%u>\n",
994                          le32_to_cpu(ndp16->dwSignature));
995                 goto err_ndp;
996         }
997         dpe16 = ndp16->dpe16;
998 
999         for (x = 0; x < nframes; x++, dpe16++) {
1000                 offset = le16_to_cpu(dpe16->wDatagramIndex);
1001                 len = le16_to_cpu(dpe16->wDatagramLength);
1002 
1003                 /*
1004                  * CDC NCM ch. 3.7
1005                  * All entries after first NULL entry are to be ignored
1006                  */
1007                 if ((offset == 0) || (len == 0)) {
1008                         if (!x)
1009                                 goto err_ndp; /* empty NTB */
1010                         break;
1011                 }
1012 
1013                 /* sanity checking */
1014                 if (((offset + len) > skb_in->len) ||
1015                                 (len > ctx->rx_max) || (len < ETH_HLEN)) {
1016                         pr_debug("invalid frame detected (ignored)"
1017                                         "offset[%u]=%u, length=%u, skb=%p\n",
1018                                         x, offset, len, skb_in);
1019                         if (!x)
1020                                 goto err_ndp;
1021                         break;
1022 
1023                 } else {
1024                         skb = skb_clone(skb_in, GFP_ATOMIC);
1025                         if (!skb)
1026                                 goto error;
1027                         skb->len = len;
1028                         skb->data = ((u8 *)skb_in->data) + offset;
1029                         skb_set_tail_pointer(skb, len);
1030                         usbnet_skb_return(dev, skb);
1031                 }
1032         }
1033 err_ndp:
1034         /* are there more NDPs to process? */
1035         ndpoffset = le16_to_cpu(ndp16->wNextNdpIndex);
1036         if (ndpoffset && loopcount--)
1037                 goto next_ndp;
1038 
1039         return 1;
1040 error:
1041         return 0;
1042 }
1043 
1044 static void
1045 cdc_ncm_speed_change(struct cdc_ncm_ctx *ctx,
1046                      struct usb_cdc_speed_change *data)
1047 {
1048         uint32_t rx_speed = le32_to_cpu(data->DLBitRRate);
1049         uint32_t tx_speed = le32_to_cpu(data->ULBitRate);
1050 
1051         /*
1052          * Currently the USB-NET API does not support reporting the actual
1053          * device speed. Do print it instead.
1054          */
1055         if ((tx_speed != ctx->tx_speed) || (rx_speed != ctx->rx_speed)) {
1056                 ctx->tx_speed = tx_speed;
1057                 ctx->rx_speed = rx_speed;
1058 
1059                 if ((tx_speed > 1000000) && (rx_speed > 1000000)) {
1060                         printk(KERN_INFO KBUILD_MODNAME
1061                                 ": %s: %u mbit/s downlink "
1062                                 "%u mbit/s uplink\n",
1063                                 ctx->netdev->name,
1064                                 (unsigned int)(rx_speed / 1000000U),
1065                                 (unsigned int)(tx_speed / 1000000U));
1066                 } else {
1067                         printk(KERN_INFO KBUILD_MODNAME
1068                                 ": %s: %u kbit/s downlink "
1069                                 "%u kbit/s uplink\n",
1070                                 ctx->netdev->name,
1071                                 (unsigned int)(rx_speed / 1000U),
1072                                 (unsigned int)(tx_speed / 1000U));
1073                 }
1074         }
1075 }
1076 
1077 static void cdc_ncm_status(struct usbnet *dev, struct urb *urb)
1078 {
1079         struct cdc_ncm_ctx *ctx;
1080         struct usb_cdc_notification *event;
1081 
1082         ctx = (struct cdc_ncm_ctx *)dev->data[0];
1083 
1084         if (urb->actual_length < sizeof(*event))
1085                 return;
1086 
1087         /* test for split data in 8-byte chunks */
1088         if (test_and_clear_bit(EVENT_STS_SPLIT, &dev->flags)) {
1089                 cdc_ncm_speed_change(ctx,
1090                       (struct usb_cdc_speed_change *)urb->transfer_buffer);
1091                 return;
1092         }
1093 
1094         event = urb->transfer_buffer;
1095 
1096         switch (event->bNotificationType) {
1097         case USB_CDC_NOTIFY_NETWORK_CONNECTION:
1098                 /*
1099                  * According to the CDC NCM specification ch.7.1
1100                  * USB_CDC_NOTIFY_NETWORK_CONNECTION notification shall be
1101                  * sent by device after USB_CDC_NOTIFY_SPEED_CHANGE.
1102                  */
1103                 ctx->connected = le16_to_cpu(event->wValue);
1104 
1105                 printk(KERN_INFO KBUILD_MODNAME ": %s: network connection:"
1106                         " %sconnected\n",
1107                         ctx->netdev->name, ctx->connected ? "" : "dis");
1108 
1109                 usbnet_link_change(dev, ctx->connected, 0);
1110                 if (!ctx->connected)
1111                         ctx->tx_speed = ctx->rx_speed = 0;
1112                 break;
1113 
1114         case USB_CDC_NOTIFY_SPEED_CHANGE:
1115                 if (urb->actual_length < (sizeof(*event) +
1116                                         sizeof(struct usb_cdc_speed_change)))
1117                         set_bit(EVENT_STS_SPLIT, &dev->flags);
1118                 else
1119                         cdc_ncm_speed_change(ctx,
1120                                 (struct usb_cdc_speed_change *) &event[1]);
1121                 break;
1122 
1123         default:
1124                 dev_dbg(&dev->udev->dev,
1125                         "NCM: unexpected notification 0x%02x!\n",
1126                         event->bNotificationType);
1127                 break;
1128         }
1129 }
1130 
1131 static int cdc_ncm_check_connect(struct usbnet *dev)
1132 {
1133         struct cdc_ncm_ctx *ctx;
1134 
1135         ctx = (struct cdc_ncm_ctx *)dev->data[0];
1136         if (ctx == NULL)
1137                 return 1;       /* disconnected */
1138 
1139         return !ctx->connected;
1140 }
1141 
1142 static int
1143 cdc_ncm_probe(struct usb_interface *udev, const struct usb_device_id *prod)
1144 {
1145         return usbnet_probe(udev, prod);
1146 }
1147 
1148 static void cdc_ncm_disconnect(struct usb_interface *intf)
1149 {
1150         struct usbnet *dev = usb_get_intfdata(intf);
1151 
1152         if (dev == NULL)
1153                 return;         /* already disconnected */
1154 
1155         usbnet_disconnect(intf);
1156 }
1157 
1158 static const struct driver_info cdc_ncm_info = {
1159         .description = "CDC NCM",
1160         .flags = FLAG_POINTTOPOINT | FLAG_NO_SETINT | FLAG_MULTI_PACKET,
1161         .bind = cdc_ncm_bind,
1162         .unbind = cdc_ncm_unbind,
1163         .check_connect = cdc_ncm_check_connect,
1164         .manage_power = usbnet_manage_power,
1165         .status = cdc_ncm_status,
1166         .rx_fixup = cdc_ncm_rx_fixup,
1167         .tx_fixup = cdc_ncm_tx_fixup,
1168 };
1169 
1170 /* Same as cdc_ncm_info, but with FLAG_WWAN */
1171 static const struct driver_info wwan_info = {
1172         .description = "Mobile Broadband Network Device",
1173         .flags = FLAG_POINTTOPOINT | FLAG_NO_SETINT | FLAG_MULTI_PACKET
1174                         | FLAG_WWAN,
1175         .bind = cdc_ncm_bind,
1176         .unbind = cdc_ncm_unbind,
1177         .check_connect = cdc_ncm_check_connect,
1178         .manage_power = usbnet_manage_power,
1179         .status = cdc_ncm_status,
1180         .rx_fixup = cdc_ncm_rx_fixup,
1181         .tx_fixup = cdc_ncm_tx_fixup,
1182 };
1183 
1184 /* Same as wwan_info, but with FLAG_NOARP  */
1185 static const struct driver_info wwan_noarp_info = {
1186         .description = "Mobile Broadband Network Device (NO ARP)",
1187         .flags = FLAG_POINTTOPOINT | FLAG_NO_SETINT | FLAG_MULTI_PACKET
1188                         | FLAG_WWAN | FLAG_NOARP,
1189         .bind = cdc_ncm_bind,
1190         .unbind = cdc_ncm_unbind,
1191         .check_connect = cdc_ncm_check_connect,
1192         .manage_power = usbnet_manage_power,
1193         .status = cdc_ncm_status,
1194         .rx_fixup = cdc_ncm_rx_fixup,
1195         .tx_fixup = cdc_ncm_tx_fixup,
1196 };
1197 
1198 static const struct usb_device_id cdc_devs[] = {
1199         /* Ericsson MBM devices like F5521gw */
1200         { .match_flags = USB_DEVICE_ID_MATCH_INT_INFO
1201                 | USB_DEVICE_ID_MATCH_VENDOR,
1202           .idVendor = 0x0bdb,
1203           .bInterfaceClass = USB_CLASS_COMM,
1204           .bInterfaceSubClass = USB_CDC_SUBCLASS_NCM,
1205           .bInterfaceProtocol = USB_CDC_PROTO_NONE,
1206           .driver_info = (unsigned long) &wwan_info,
1207         },
1208 
1209         /* Dell branded MBM devices like DW5550 */
1210         { .match_flags = USB_DEVICE_ID_MATCH_INT_INFO
1211                 | USB_DEVICE_ID_MATCH_VENDOR,
1212           .idVendor = 0x413c,
1213           .bInterfaceClass = USB_CLASS_COMM,
1214           .bInterfaceSubClass = USB_CDC_SUBCLASS_NCM,
1215           .bInterfaceProtocol = USB_CDC_PROTO_NONE,
1216           .driver_info = (unsigned long) &wwan_info,
1217         },
1218 
1219         /* Toshiba branded MBM devices */
1220         { .match_flags = USB_DEVICE_ID_MATCH_INT_INFO
1221                 | USB_DEVICE_ID_MATCH_VENDOR,
1222           .idVendor = 0x0930,
1223           .bInterfaceClass = USB_CLASS_COMM,
1224           .bInterfaceSubClass = USB_CDC_SUBCLASS_NCM,
1225           .bInterfaceProtocol = USB_CDC_PROTO_NONE,
1226           .driver_info = (unsigned long) &wwan_info,
1227         },
1228 
1229         /* tag Huawei devices as wwan */
1230         { USB_VENDOR_AND_INTERFACE_INFO(0x12d1,
1231                                         USB_CLASS_COMM,
1232                                         USB_CDC_SUBCLASS_NCM,
1233                                         USB_CDC_PROTO_NONE),
1234           .driver_info = (unsigned long)&wwan_info,
1235         },
1236 
1237         /* Huawei NCM devices disguised as vendor specific */
1238         { USB_VENDOR_AND_INTERFACE_INFO(0x12d1, 0xff, 0x02, 0x16),
1239           .driver_info = (unsigned long)&wwan_info,
1240         },
1241         { USB_VENDOR_AND_INTERFACE_INFO(0x12d1, 0xff, 0x02, 0x46),
1242           .driver_info = (unsigned long)&wwan_info,
1243         },
1244         { USB_VENDOR_AND_INTERFACE_INFO(0x12d1, 0xff, 0x02, 0x76),
1245           .driver_info = (unsigned long)&wwan_info,
1246         },
1247 
1248         /* Infineon(now Intel) HSPA Modem platform */
1249         { USB_DEVICE_AND_INTERFACE_INFO(0x1519, 0x0443,
1250                 USB_CLASS_COMM,
1251                 USB_CDC_SUBCLASS_NCM, USB_CDC_PROTO_NONE),
1252           .driver_info = (unsigned long)&wwan_noarp_info,
1253         },
1254 
1255         /* Generic CDC-NCM devices */
1256         { USB_INTERFACE_INFO(USB_CLASS_COMM,
1257                 USB_CDC_SUBCLASS_NCM, USB_CDC_PROTO_NONE),
1258                 .driver_info = (unsigned long)&cdc_ncm_info,
1259         },
1260         {
1261         },
1262 };
1263 MODULE_DEVICE_TABLE(usb, cdc_devs);
1264 
1265 static struct usb_driver cdc_ncm_driver = {
1266         .name = "cdc_ncm",
1267         .id_table = cdc_devs,
1268         .probe = cdc_ncm_probe,
1269         .disconnect = cdc_ncm_disconnect,
1270         .suspend = usbnet_suspend,
1271         .resume = usbnet_resume,
1272         .reset_resume = usbnet_resume,
1273         .supports_autosuspend = 1,
1274         .disable_hub_initiated_lpm = 1,
1275 };
1276 
1277 module_usb_driver(cdc_ncm_driver);
1278 
1279 MODULE_AUTHOR("Hans Petter Selasky");
1280 MODULE_DESCRIPTION("USB CDC NCM host driver");
1281 MODULE_LICENSE("Dual BSD/GPL");
1282 

This page was automatically generated by LXR 0.3.1 (source).  •  Linux is a registered trademark of Linus Torvalds  •  Contact us

    Home
    Development
    Services
    Training
    Docs
    Community
    Company
    Blog


