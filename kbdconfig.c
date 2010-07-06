#include <libusb-1.0/libusb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "usbconfig.h"

#define HID_GET_REPORT 0x01
#define HID_SET_REPORT 0x09
#define HID_REPORT_TYPE_FEATURE 0x03

#define REPORT_ID 0x00

static unsigned char usb_cfg_vendor_id[2] = { USB_CFG_VENDOR_ID };
static unsigned char usb_cfg_device_id[2] = { USB_CFG_DEVICE_ID };

int main(int argc, char **argv)
{
    int r;
    libusb_device_handle * dev;
    unsigned char buffer[1];

    libusb_init(NULL);

    dev = libusb_open_device_with_vid_pid
        (NULL,
         usb_cfg_vendor_id[1] << 8 | usb_cfg_vendor_id[0],
         usb_cfg_device_id[1] << 8 | usb_cfg_device_id[0]);
    if (dev)
    {
        printf("device found\n");

        buffer[0] = 0xff;
        r = libusb_control_transfer
            (dev,
             LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_DEVICE,
             HID_GET_REPORT, (HID_REPORT_TYPE_FEATURE<<8)|REPORT_ID, 0, buffer, 1, 1000);
        if (r < 0)
        {
            fprintf(stderr, "Control In error %d\n", r);
        }

        printf("received: 0x%02x\n", buffer[0]);

        if (argc > 1)
        {
            *buffer = atoi(argv[1]);
            printf("sent: 0x%02x\n", buffer[0]);

            r = libusb_control_transfer
                (dev,
                 LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_DEVICE,
                 HID_SET_REPORT, (HID_REPORT_TYPE_FEATURE<<8)|REPORT_ID, 0, buffer, 1, 1000);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
            }

            buffer[0] = 0xff;
            r = libusb_control_transfer
                (dev,
                 LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_DEVICE,
                 HID_GET_REPORT, (HID_REPORT_TYPE_FEATURE<<8)|REPORT_ID, 0, buffer, 1, 1000);
            if (r < 0)
            {
                fprintf(stderr, "Control In error %d\n", r);
            }

            printf("received: 0x%02x\n", buffer[0]);

        }

        libusb_close(dev);
    }

    libusb_exit(NULL);
    return 0;
}
