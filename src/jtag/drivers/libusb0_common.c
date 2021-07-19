/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "log.h"
#include "libusb0_common.h"

static bool jtag_libusb_match(struct jtag_libusb_device *dev,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev->descriptor.idVendor == vids[i] &&
			dev->descriptor.idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

#if (_NDS32_ONLY_ == 0)
/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(usb_dev_handle *device, uint8_t str_index,
									const char *string)
{
	int retval;
	bool matched;
	char desc_string[256+1]; /* Max size of string descriptor */

	if (str_index == 0)
		return false;

	retval = usb_get_string_simple(device, str_index,
			desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("usb_get_string_simple() failed with %d", retval);
		return false;
	}

	/* Null terminate descriptor string in case it needs to be logged. */
	desc_string[sizeof(desc_string)-1] = '\0';

	matched = strncmp(string, desc_string, sizeof(desc_string)) == 0;
	if (!matched)
		LOG_DEBUG("Device serial number '%s' doesn't match requested serial '%s'",
			desc_string, string);
	return matched;
}
#endif /* (_NDS32_ONLY_ == 0) */

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct jtag_libusb_device_handle **out)
{
	int retval = ERROR_FAIL;
	bool serial_mismatch = false;
	struct jtag_libusb_device_handle *libusb_handle;
	usb_init();

	usb_find_busses();
	usb_find_devices();

	struct usb_bus *busses = usb_get_busses();
	for (struct usb_bus *bus = busses; bus; bus = bus->next) {
		for (struct usb_device *dev = bus->devices;
				dev; dev = dev->next) {
			if (!jtag_libusb_match(dev, vids, pids))
				continue;

#if _NDS32_ONLY_
			libusb_handle = usb_open(dev);
			if (NULL == libusb_handle)
				return -errno;
			/* claim usb interface, if fail, search the next device. (for multi-AICEs support) */
			/* set_configuration must be before claim_interface() (RedHat64) */
			retval = jtag_libusb_set_configuration(libusb_handle, 0);
			retval = jtag_libusb_claim_interface(libusb_handle, 0);
			*out = libusb_handle;
			if (retval == 0)
				return 0;
#else /* _NDS32_ONLY_ */
			libusb_handle = usb_open(dev);
			if (NULL == libusb_handle) {
				LOG_ERROR("usb_open() failed with %s", usb_strerror());
				continue;
			}

			/* Device must be open to use libusb_get_string_descriptor_ascii. */
			if (serial != NULL &&
					!string_descriptor_equal(libusb_handle, dev->descriptor.iSerialNumber, serial)) {
				serial_mismatch = true;
				usb_close(libusb_handle);
				continue;
			}
			*out = libusb_handle;
			retval = ERROR_OK;
			serial_mismatch = false;
			break;
#endif /* _NDS32_ONLY_ */
		}
	}

	if (serial_mismatch)
		LOG_INFO("No device matches the serial string");

	return retval;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	/* Close device */
	usb_close(dev);
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = usb_control_msg(dev, requestType, request, wValue, wIndex,
				bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	return usb_bulk_write(dev, ep, bytes, size, timeout);
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	return usb_bulk_read(dev, ep, bytes, size, timeout);
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);

	return usb_set_configuration(devh,
			udev->config[configuration].bConfigurationValue);
}

int jtag_libusb_choose_interface(struct jtag_libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	struct usb_interface *iface = udev->config->interface;
	struct usb_interface_descriptor *desc = iface->altsetting;

	*usb_read_ep = *usb_write_ep = 0;

	for (int i = 0; i < desc->bNumEndpoints; i++) {
		if ((bclass > 0 && desc->bInterfaceClass != bclass) ||
		    (subclass > 0 && desc->bInterfaceSubClass != subclass) ||
		    (protocol > 0 && desc->bInterfaceProtocol != protocol) ||
		    (trans_type > 0 && (desc->endpoint[i].bmAttributes & 0x3) != trans_type))
			continue;

		uint8_t epnum = desc->endpoint[i].bEndpointAddress;
		bool is_input = epnum & 0x80;
		LOG_DEBUG("usb ep %s %02x", is_input ? "in" : "out", epnum);
		if (is_input)
			*usb_read_ep = epnum;
		else
			*usb_write_ep = epnum;

		if (*usb_read_ep && *usb_write_ep) {
			LOG_DEBUG("Claiming interface %d", (int)desc->bInterfaceNumber);
			usb_claim_interface(devh, (int)desc->bInterfaceNumber);
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct jtag_libusb_device *dev, uint16_t *pid)
{
	if (!dev)
		return ERROR_FAIL;

	*pid = dev->descriptor.idProduct;
	return ERROR_OK;
}

#if _NDS32_ONLY_
int jtag_libusb_get_endpoints(struct jtag_libusb_device *udev,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		unsigned int *usb_rx_max_packet,
		unsigned int *usb_tx_max_packet)
{
	struct usb_interface *iface = udev->config->interface;
	struct usb_interface_descriptor *desc = iface->altsetting;

	for (int i = 0; i < desc->bNumEndpoints; i++) {
		if (desc->endpoint[i].bmAttributes != USB_ENDPOINT_TYPE_BULK)
			continue;

		uint8_t epnum = desc->endpoint[i].bEndpointAddress;
		bool is_input = epnum & 0x80;

		if (is_input) {
			*usb_read_ep = epnum;
			*usb_rx_max_packet = desc->endpoint[i].wMaxPacketSize;
		} else {
			*usb_write_ep = epnum;
			*usb_tx_max_packet = desc->endpoint[i].wMaxPacketSize;
		}
	}

	return 0;
}

unsigned char descriptor_string_iManufacturer[128];
unsigned char descriptor_string_iProduct[128];
unsigned int descriptor_bcdDevice = 0x0;
char descriptor_string_unknown[] = {"unknown"};

int jtag_libusb_get_descriptor_string(jtag_libusb_device_handle *dev_handle,
		struct jtag_libusb_device *dev,
		char **pdescp_Manufacturer,
		char **pdescp_Product,
		unsigned int *pdescp_bcdDevice)
{
	int ret1, ret2;
	char *pStringManufacturer, *pStringProduct;

	ret1 = usb_get_string_simple(dev_handle, dev->descriptor.iManufacturer,
		(char *)&descriptor_string_iManufacturer[0], sizeof(descriptor_string_iManufacturer)-1);
	ret2 = usb_get_string_simple(dev_handle, dev->descriptor.iProduct,
		(char *)&descriptor_string_iProduct[0], sizeof(descriptor_string_iProduct)-1);

	pStringManufacturer = (char *)&descriptor_string_unknown[0];
	pStringProduct = (char *)&descriptor_string_unknown[0];
	if (ret1 > 0)
		pStringManufacturer = (char *)&descriptor_string_iManufacturer[0];
	if (ret2 > 0)
		pStringProduct = (char *)&descriptor_string_iProduct[0];
	*pdescp_Manufacturer = pStringManufacturer;
	*pdescp_Product = pStringProduct;
	*pdescp_bcdDevice = (unsigned int)dev->descriptor.bcdDevice;
	return 0;
}
#endif /* _NDS32_ONLY_ */

