#include <libusb-1.0/libusb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../firmware/usbconfig.h"

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
    unsigned char buffer[4];

    if (argc < 2 || argc > 3)
    {
        printf("usage: %s <config>\n"
               "       %s <key code> <hid code>\n"
               "       %s bootloader"
               "\n"
               "  config: bits 0-1: key click mode:\n"
               "                     0: off\n"
               "                     1: on\n"
               "                     2: caps on\n"
               "          bit 2: save config to EEPROM\n"
               "          bit 3: read config from EEPROM\n", argv[0], argv[0], argv[0]);
        exit(1);
    }

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

        switch(argc)
        {
        case 2:
            if (strcmp(argv[1], "bootloader") == 0)
            {
                buffer[0] = 3; /* FEATURE_BOOTLOADER */
                buffer[1] = 0; /* reserved */

                r = libusb_control_transfer
                    (dev,
                     LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_DEVICE,
                     HID_SET_REPORT, (HID_REPORT_TYPE_FEATURE<<8)|REPORT_ID, 0, buffer, 2, 1000);
                if (r < 0)
                {
                    fprintf(stderr, "Control Out error %d\n", r);
                }
            }
            else
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
            break;

        case 3:
            buffer[0] = 2; /* FEATURE_OVERRIDE_KEYCODE */
            buffer[1] = 0; /* reserved */
            buffer[2] = atoi(argv[1]); /* key code */
            buffer[3] = atoi(argv[2]); /* HID code */

            r = libusb_control_transfer
                (dev,
                 LIBUSB_ENDPOINT_OUT|LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_DEVICE,
                 HID_SET_REPORT, (HID_REPORT_TYPE_FEATURE<<8)|REPORT_ID, 0, buffer, 4, 1000);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
            }
        }


        libusb_close(dev);
    }

    libusb_exit(NULL);
    return 0;
}
