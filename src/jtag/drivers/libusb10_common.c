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
#include "libusb10_common.h"

static struct libusb_context *jtag_libusb_context; /**< Libusb context **/
static libusb_device **devs; /**< The usb device list **/

static bool jtag_libusb_match(struct libusb_device_descriptor *dev_desc,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev_desc->idVendor == vids[i] &&
			dev_desc->idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

int use_libusb0; /* Use libusb1 first */
#define LIBUSB0_PATH_MAX 512

#ifdef USE_LIBUSB0
use_libusb0 = 1;
#endif

struct libusb0_device;
struct libusb0_bus {
	struct libusb0_bus *next, *prev;
	char dirname[LIBUSB0_PATH_MAX];
	struct libusb0_device *devices;
	unsigned long location;
	struct libusb0_device *root_dev;
};
struct libusb0_device {
	struct libusb0_device *next, *prev;
	char filename[LIBUSB0_PATH_MAX];
	struct libusb0_bus *bus;
	struct libusb_device_descriptor descriptor;
	struct libusb_config_descriptor *config;
	void *dev;          /* Darwin support */
	unsigned char devnum;
	unsigned char num_children;
	struct libusb0_device **children;
} __attribute__((packed));


typedef void (*libusb0_init_t)(void);
typedef int (*libusb0_int_void_t)(void);
typedef struct libusb0_bus * (*libusb0_get_busses_t)(void);
typedef libusb_device_handle * (*libusb0_open_t)(struct libusb0_device *dev);
typedef int (*libusb0_set_configuration_t)(libusb_device_handle *dev, int configuration);
typedef int (*libusb0_claim_interface_t)(libusb_device_handle *dev, int interface);
typedef int (*libusb0_close_t)(libusb_device_handle *dev);
typedef int (*libusb0_reset_t)(libusb_device_handle *dev);
typedef int (*libusb0_clear_halt_t)(libusb_device_handle *dev, unsigned int ep);
typedef char *(*libusb0_strerror_t)(void);
typedef int (*libusb0_control_msg_t)(libusb_device_handle *dev, int requesttype,
		int request, int value, int index, char *bytes, int size, int timeout);
typedef int (*libusb0_bulk_read_t)(libusb_device_handle *dev, int ep, char *bytes, int size, int timeout);
typedef int (*libusb0_bulk_write_t)(libusb_device_handle *dev, int ep, char *bytes, int size, int timeout);
typedef int (*libusb0_release_interface_t)(libusb_device_handle *dev, int interface);
typedef int (*libusb0_get_descriptor_t)(libusb_device_handle *dev,
		unsigned char type, unsigned char index, void *buf, int size);
typedef int (*libusb0_get_string_simple_t)(libusb_device_handle *dev, int index, char *buf, size_t buflen);

libusb0_init_t libusb0_init;
libusb0_int_void_t libusb0_find_busses;
libusb0_int_void_t libusb0_find_devices;
libusb0_get_busses_t libusb0_get_busses;
libusb0_open_t libusb0_open;
libusb0_set_configuration_t libusb0_set_configuration;
libusb0_claim_interface_t libusb0_claim_interface;
libusb0_close_t libusb0_close;
libusb0_reset_t libusb0_reset;
libusb0_clear_halt_t libusb0_clear_halt;
libusb0_strerror_t libusb0_strerror;
libusb0_control_msg_t libusb0_control_msg;
libusb0_bulk_read_t libusb0_bulk_read;
libusb0_bulk_write_t libusb0_bulk_write;
libusb0_release_interface_t libusb0_release_interface;
libusb0_get_descriptor_t libusb0_get_descriptor;
libusb0_get_string_simple_t libusb0_get_string_simple;

static char *libusb0_dll_name = "libusb0";
struct libusb_device_handle *libusb0_handle;
struct libusb0_device *libusb0_dev;

void *libusb0_find_func(HMODULE h, char *name)
{
	void *proc = GetProcAddress(h, name);
	if (proc == NULL) {
		printf("error: cannot find %s() in %s.dll\n", name, libusb0_dll_name);
		exit(1);
	}
	return proc;
}

int load_libusb_dll(void)
{
	HMODULE h = NULL;

	h = GetModuleHandleA(libusb0_dll_name);
	if (h == NULL)
		h = LoadLibraryA(libusb0_dll_name);

	if (h == NULL)
		return ERROR_FAIL;

	libusb0_init = (libusb0_init_t) libusb0_find_func(h, "usb_init");
	libusb0_find_busses = (libusb0_int_void_t) libusb0_find_func(h, "usb_find_busses");
	libusb0_find_devices = (libusb0_int_void_t) libusb0_find_func(h, "usb_find_devices");
	libusb0_get_busses = (libusb0_get_busses_t) libusb0_find_func(h, "usb_get_busses");
	libusb0_open = (libusb0_open_t) libusb0_find_func(h, "usb_open");
	libusb0_set_configuration = (libusb0_set_configuration_t) libusb0_find_func(h, "usb_set_configuration");
	libusb0_claim_interface = (libusb0_claim_interface_t) libusb0_find_func(h, "usb_claim_interface");
	libusb0_close = (libusb0_close_t) libusb0_find_func(h, "usb_close");
	libusb0_reset = (libusb0_reset_t) libusb0_find_func(h, "usb_reset");
	libusb0_clear_halt = (libusb0_clear_halt_t) libusb0_find_func(h, "usb_clear_halt");
	libusb0_strerror = (libusb0_strerror_t) libusb0_find_func(h, "usb_strerror");
	libusb0_control_msg = (libusb0_control_msg_t) libusb0_find_func(h, "usb_control_msg");
	libusb0_bulk_read = (libusb0_bulk_read_t) libusb0_find_func(h, "usb_bulk_read");
	libusb0_bulk_write = (libusb0_bulk_write_t) libusb0_find_func(h, "usb_bulk_write");
	libusb0_release_interface = (libusb0_release_interface_t) libusb0_find_func(h, "usb_release_interface");
	libusb0_get_descriptor = (libusb0_get_descriptor_t) libusb0_find_func(h, "usb_get_descriptor");
	libusb0_get_string_simple = (libusb0_get_string_simple_t) libusb0_find_func(h, "usb_get_string_simple");
	return ERROR_OK;
}

static bool jtag_libusb0_match(struct libusb0_device *dev,
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

int jtag_libusb0_open(const uint16_t vids[], const uint16_t pids[])
{
	if (libusb0_init == NULL) {
		if (load_libusb_dll() < 0) {
			LOG_ERROR("libusb0 dll load failed");
			return ERROR_FAIL;
		}

		libusb0_init();
	}

	libusb0_find_busses();
	libusb0_find_devices();

	struct libusb0_bus *busses = libusb0_get_busses();
	for (struct libusb0_bus *bus = busses; bus; bus = bus->next) {
		for (struct libusb0_device *dev = bus->devices;
				dev; dev = dev->next) {

			if (!jtag_libusb0_match(dev, vids, pids))
				continue;

			libusb0_handle = libusb0_open(dev);
			if (NULL == libusb0_handle)
				return -errno;

			if (libusb0_set_configuration(libusb0_handle, 1) < 0)
				continue;

			if (libusb0_claim_interface(libusb0_handle, 0) < 0) {
				libusb0_close(libusb0_handle);
				continue;
			}

			use_libusb0 = 1;
			libusb0_dev = dev;
			LOG_DEBUG("use_libusb0 = 1");
			return ERROR_OK;
		}
	}

	LOG_ERROR("libusb0 open failed!!");
	return ERROR_FAIL;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial,
		struct jtag_libusb_device_handle **out)
{

	if (use_libusb0 == 1)
		return jtag_libusb0_open(vids, pids);

	int cnt, idx, errCode;
	int retval = -ENODEV;
	struct jtag_libusb_device_handle *libusb_handle = NULL;

	if (libusb_init(&jtag_libusb_context) < 0)
		return -ENODEV;

	cnt = libusb_get_device_list(jtag_libusb_context, &devs);

	for (idx = 0; idx < cnt; idx++) {
		struct libusb_device_descriptor dev_desc;

		if (libusb_get_device_descriptor(devs[idx], &dev_desc) != 0)
			continue;

		if (!jtag_libusb_match(&dev_desc, vids, pids))
			continue;

		errCode = libusb_open(devs[idx], &libusb_handle);

		if (errCode) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(errCode));
			continue;
		}

		/* set_configuration must be before claim_interface() (RedHat64) */
		retval = jtag_libusb_set_configuration(libusb_handle, 0);
		retval = libusb_claim_interface(libusb_handle, 0);
		if (retval == 0) {
			*out = libusb_handle;
			/** Free the device list **/
			libusb_free_device_list(devs, 1);
			return 0;
		}
	}
	if (cnt >= 0)
		libusb_free_device_list(devs, 1);

	if (retval != 0)
		return jtag_libusb0_open(vids, pids);

	return retval;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	if (use_libusb0) {
		if (libusb0_handle != NULL) {
			libusb0_release_interface(libusb0_handle, 0);
			libusb0_close(libusb0_handle);
		}
		libusb0_handle = NULL;
		return;
	} else {
		/* Close device */
		libusb_close(dev);
		libusb_exit(jtag_libusb_context);
	}
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	if (use_libusb0) {
		transferred = libusb0_control_msg(libusb0_handle, requestType, request, wValue, wIndex,
			bytes, size, timeout);
	} else {
		transferred = libusb_control_transfer(dev, requestType, request, wValue, wIndex,
					(unsigned char *)bytes, size, timeout);
	}

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{

	if (use_libusb0) {
		return libusb0_bulk_write(libusb0_handle, ep, bytes, size, timeout);
	} else {
		int transferred = 0;

		libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				     &transferred, timeout);
		return transferred;
	}
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	if (use_libusb0) {
		return libusb0_bulk_read(libusb0_handle, ep, bytes, size, timeout);
	} else {
		int transferred = 0;

		libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				     &transferred, timeout);
		return transferred;
	}
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	if (use_libusb0) {
		/* TODO: ERROR HERE!!!!! */
		/*
		 * return libusb0_set_configuration(libusb0_handle, libusb0_handle->config[configuration].bConfigurationValue);
		 */
		return 0;
	} else {
		struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
		int retCode = -99;

		struct libusb_config_descriptor *config = NULL;
		int current_config = -1;

		retCode = libusb_get_configuration(devh, &current_config);
		if (retCode != 0)
			return retCode;

		retCode = libusb_get_config_descriptor(udev, configuration, &config);
		if (retCode != 0 || config == NULL)
			return retCode;

		/* Only change the configuration if it is not already set to the
		   same one. Otherwise this issues a lightweight reset and hangs
		   LPC-Link2 with JLink firmware. */
		if (current_config != config->bConfigurationValue)
			retCode = libusb_set_configuration(devh, config->bConfigurationValue);

		libusb_free_config_descriptor(config);

		return retCode;
	}
}

static void aice_usb_parse_descriptor(unsigned char *src, char *desc, void *dst)
{
	unsigned short int w;
	unsigned char ch;
	while ((ch = *desc++)) {
		switch (ch) {
			case 'b': /* byte */
				*(unsigned char *)dst++ = *src++;
				break;

			case 'w': /* short */
				w = src[0] | (src[1]<<8);
				src += 2;
				dst += ((unsigned int)dst & 1); /* align to half-word boundary */
				memcpy(dst, &w, sizeof(w));
				dst += 2;
				break;
		};
	}
}

int jtag_libusb_choose_interface(struct jtag_libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type)
{


	if (use_libusb0) {
		*usb_read_ep = *usb_write_ep = 0;

		const int DESC_BUF_SIZE = 512;
		unsigned char *buf;
		unsigned char buffer[DESC_BUF_SIZE];
		struct libusb_device_descriptor desc;
		int ret;

		buf = buffer;
		ret = libusb0_get_descriptor(libusb0_handle, LIBUSB_DT_DEVICE, 0, buf, LIBUSB_DT_DEVICE_SIZE);
		if (ret < 0)
			return ERROR_FAIL;

		aice_usb_parse_descriptor(buf, "bbwbbbbwwwbbbb", &desc);
		for (int c = 0; c < desc.bNumConfigurations; c++) {
			struct libusb_config_descriptor config;
			ret = libusb0_get_descriptor(libusb0_handle, LIBUSB_DT_CONFIG, 0, buf, DESC_BUF_SIZE);
			if (ret < 0)
				return ERROR_FAIL;

			aice_usb_parse_descriptor(buf, "bbwbbbbb", &config);
			buf += config.bLength;

			for (int i = 0; i < config.bNumInterfaces; i++) {
				struct libusb_interface_descriptor interface;

				while (buf[1] != LIBUSB_DT_INTERFACE) {   /* bDescriptorType */
					buf += (unsigned int) buf[0];     /* bLength */
				}
				aice_usb_parse_descriptor(buf, "bbbbbbbbb", &interface);
				buf += interface.bLength;

				for (int e = 0; e < interface.bNumEndpoints; e++) {
					struct libusb_endpoint_descriptor endpoint;
					while (buf[1] != LIBUSB_DT_ENDPOINT) {    /* bDescriptorType */
						buf += (unsigned int) buf[0];     /* bLength */
					}

					aice_usb_parse_descriptor(buf, "bbbbwb", &endpoint);
					buf += endpoint.bLength;

					if ((endpoint.bmAttributes & 0x3) == 0x2) {	/* BULK */
						uint8_t epnum = endpoint.bEndpointAddress;
						bool is_input = epnum & 0x80;
						LOG_DEBUG("usb ep %s %02x", is_input ? "in" : "out", epnum);
						if (is_input)
							*usb_read_ep = epnum;
						else
							*usb_write_ep = epnum;
					}

					if (*usb_read_ep && *usb_write_ep) {
						LOG_DEBUG("Find ep!!");
						return ERROR_OK;
					}
				}
			}
		}
	} else {
		struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
		const struct libusb_interface *inter;
		const struct libusb_interface_descriptor *interdesc;
		const struct libusb_endpoint_descriptor *epdesc;
		struct libusb_config_descriptor *config;

		*usb_read_ep = *usb_write_ep = 0;

		libusb_get_config_descriptor(udev, 0, &config);
		for (int i = 0; i < (int)config->bNumInterfaces; i++) {
			inter = &config->interface[i];

			interdesc = &inter->altsetting[0];
			for (int k = 0;
			     k < (int)interdesc->bNumEndpoints; k++) {
				if ((bclass > 0 && interdesc->bInterfaceClass != bclass) ||
				    (subclass > 0 && interdesc->bInterfaceSubClass != subclass) ||
				    (protocol > 0 && interdesc->bInterfaceProtocol != protocol))
					continue;

				epdesc = &interdesc->endpoint[k];
				if (trans_type > 0 && (epdesc->bmAttributes & 0x3) != trans_type)
					continue;

				uint8_t epnum = epdesc->bEndpointAddress;
				bool is_input = epnum & 0x80;
				LOG_DEBUG("usb ep %s %02x",
					  is_input ? "in" : "out", epnum);

				if (is_input)
					*usb_read_ep = epnum;
				else
					*usb_write_ep = epnum;

				if (*usb_read_ep && *usb_write_ep) {
					LOG_DEBUG("Claiming interface %d", (int)interdesc->bInterfaceNumber);
					libusb_claim_interface(devh, (int)interdesc->bInterfaceNumber);
					libusb_free_config_descriptor(config);
					return ERROR_OK;
				}
			}
		}
		libusb_free_config_descriptor(config);
	}

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct jtag_libusb_device *dev, uint16_t *pid)
{
	if (use_libusb0) {
		if (!libusb0_handle)
			return ERROR_FAIL;

		*pid = libusb0_dev->descriptor.idProduct;
		return ERROR_OK;
	} else {
		struct libusb_device_descriptor dev_desc;

		if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
			*pid = dev_desc.idProduct;

			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

int jtag_libusb_get_endpoints(struct jtag_libusb_device *udev,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		unsigned int *usb_rx_max_packet,
		unsigned int *usb_tx_max_packet)
{
	if (use_libusb0) {
		const int DESC_BUF_SIZE = 512;
		unsigned char *buf;
		unsigned char buffer[DESC_BUF_SIZE];
		struct libusb_device_descriptor desc;
		int ret;

		buf = buffer;
		ret = libusb0_get_descriptor(libusb0_dev, LIBUSB_DT_DEVICE, 0, buf, LIBUSB_DT_DEVICE_SIZE);
		if (ret < 0)
			return ERROR_FAIL;

		aice_usb_parse_descriptor(buf, "bbwbbbbwwwbbbb", &desc);
		for (int c = 0; c < desc.bNumConfigurations; c++) {
			struct libusb_config_descriptor config;
			ret = libusb0_get_descriptor(libusb0_handle, LIBUSB_DT_CONFIG, 0, buf, DESC_BUF_SIZE);
			if (ret < 0)
				return ERROR_FAIL;

			aice_usb_parse_descriptor(buf, "bbwbbbbb", &config);
			buf += config.bLength;

			for (int i = 0; i < config.bNumInterfaces; i++) {
				struct libusb_interface_descriptor interface;

				while (buf[1] != LIBUSB_DT_INTERFACE) {   /* bDescriptorType */
					buf += (unsigned int) buf[0];     /* bLength */
				}
				aice_usb_parse_descriptor(buf, "bbbbbbbbb", &interface);
				buf += interface.bLength;

				for (int e = 0; e < interface.bNumEndpoints; e++) {
					struct libusb_endpoint_descriptor endpoint;
					while (buf[1] != LIBUSB_DT_ENDPOINT) {    /* bDescriptorType */
						buf += (unsigned int) buf[0];     /* bLength */
					}

					aice_usb_parse_descriptor(buf, "bbbbwb", &endpoint);
					buf += endpoint.bLength;

					if ((endpoint.bmAttributes & 0x3) != 0x2)
						continue;

					uint8_t epnum = endpoint.bEndpointAddress;
					bool is_input = epnum & 0x80;

					if (is_input) {
						*usb_read_ep = epnum;
						*usb_rx_max_packet = endpoint.wMaxPacketSize;
					} else {
						*usb_write_ep = epnum;
						*usb_tx_max_packet = endpoint.wMaxPacketSize;
					}
				}

			}
		}
	} else {
		const struct libusb_interface *inter;
		const struct libusb_interface_descriptor *interdesc;
		const struct libusb_endpoint_descriptor *epdesc;
		struct libusb_config_descriptor *config;

		libusb_get_config_descriptor(udev, 0, &config);
		for (int i = 0; i < (int)config->bNumInterfaces; i++) {
			inter = &config->interface[i];

			for (int j = 0; j < inter->num_altsetting; j++) {
				interdesc = &inter->altsetting[j];
				for (int k = 0;
					k < (int)interdesc->bNumEndpoints; k++) {
					epdesc = &interdesc->endpoint[k];
					if (epdesc->bmAttributes != LIBUSB_TRANSFER_TYPE_BULK)
						continue;

					uint8_t epnum = epdesc->bEndpointAddress;
					bool is_input = epnum & 0x80;

					if (is_input) {
						*usb_read_ep = epnum;
						*usb_rx_max_packet = epdesc->wMaxPacketSize;
					} else {
						*usb_write_ep = epnum;
						*usb_tx_max_packet = epdesc->wMaxPacketSize;
					}
				}
			}
		}
		libusb_free_config_descriptor(config);
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
	if (use_libusb0) {
		ret1 = libusb0_get_string_simple(libusb0_handle, libusb0_dev->descriptor.iManufacturer,
			(char *)&descriptor_string_iManufacturer[0], sizeof(descriptor_string_iManufacturer)-1);
		ret2 = libusb0_get_string_simple(libusb0_handle, libusb0_dev->descriptor.iProduct,
			(char *)&descriptor_string_iProduct[0], sizeof(descriptor_string_iProduct)-1);

		pStringManufacturer = (char *)&descriptor_string_unknown[0];
		pStringProduct = (char *)&descriptor_string_unknown[0];
		if (ret1 > 0)
			pStringManufacturer = (char *)&descriptor_string_iManufacturer[0];

		if (ret2 > 0)
			pStringProduct = (char *)&descriptor_string_iProduct[0];

		*pdescp_Manufacturer = pStringManufacturer;
		*pdescp_Product = pStringProduct;
		*pdescp_bcdDevice = (unsigned int)libusb0_dev->descriptor.bcdDevice;
		return 0;
	} else {
		struct libusb_device_descriptor dev_desc;
		libusb_get_device_descriptor(dev, &dev_desc);

		ret1 = libusb_get_string_descriptor_ascii(dev_handle, dev_desc.iManufacturer,
			&descriptor_string_iManufacturer[0], sizeof(descriptor_string_iManufacturer)-1);
		ret2 = libusb_get_string_descriptor_ascii(dev_handle, dev_desc.iProduct,
			&descriptor_string_iProduct[0], sizeof(descriptor_string_iProduct)-1);

		pStringManufacturer = (char *)&descriptor_string_unknown[0];
		pStringProduct = (char *)&descriptor_string_unknown[0];
		if (ret1 > 0)
			pStringManufacturer = (char *)&descriptor_string_iManufacturer[0];

		if (ret2 > 0)
			pStringProduct = (char *)&descriptor_string_iProduct[0];

		*pdescp_Manufacturer = pStringManufacturer;
		*pdescp_Product = pStringProduct;
		*pdescp_bcdDevice = (unsigned int)dev_desc.bcdDevice;
		return 0;
	}
}

struct jtag_libusb_device *jtag_libusb_get_device(struct jtag_libusb_device_handle *dev_handle)
{
	if (use_libusb0)
		return NULL;
	else
		return libusb_get_device(dev_handle);
}

