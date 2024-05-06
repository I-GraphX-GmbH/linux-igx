// SPDX-License-Identifier: GPL-2.0-only
/*
 *  max14916 - 8 Pin GPIO driver
 *
 *  Copyright (C) 2022 I-GraphX GmbH, Philipp Harms <philipp.harms@i-graphx.com>
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>


#define MAX14916_NUMBER_GPIOS	 8

#define MAX14916_REG_SET_OUT      0x00
#define MAX14916_REG_SET_FLED     0x01
#define MAX14916_REG_SET_SLED     0x02
#define MAX14916_REG_IRQ          0x03
#define MAX14916_REG_OVL_CH_F     0x04
#define MAX14916_REG_CURR_LIM     0x05
#define MAX14916_REG_OW_OFF_CH_F  0x06
#define MAX14916_REG_OW_ON_CH_F   0x07
#define MAX14916_REG_SHT_VDD_CH_F 0x08
#define MAX14916_REG_GLOBAL_ERR   0x09
#define MAX14916_REG_OW_OFF_EN    0x0a
#define MAX14916_REG_OW_ON_EN     0x0b
#define MAX14916_REG_SHT_VDD_EN   0x0c
#define MAX14916_REG_CONFIG_1     0x0d
#define MAX14916_REG_CONFIG_2     0x0e
#define MAX14916_REG_MASK         0x0f

#define MAX14916_DT_PROP_CRC	  "maxim,crc-en"
#define MAX14916_DT_PROP_DAISY	  "#daisy-chained-devices"
#define MAX14916_DT_PROP_ADDRESS  "maxim,address-bits"
#define MAX14916_DT_PROP_AMODE    "maxim,addressed-mode"

enum max14916_mode {
	DAISY_CHAIN,
	ADDRESS_BITS,
};

struct max14916_chip {
	struct gpio_chip	gpio_chip;
	struct mutex		lock;
	struct gpio_desc	*gpiod_oe;
	u32			ndaisy_chained;
	bool			address_bit_0; // A0
	bool			address_bit_1; // A1
	bool                    crc_en;
	bool			burst;
	enum max14916_mode	mode;
	u8			pin_buffer[];
};

static u8 __max14916_get_data_length(struct max14916_chip *chip, bool write, bool three_regs_burst) {
	if(chip->mode == DAISY_CHAIN) {
		return (chip->crc_en ? 2 : 1) * chip->ndaisy_chained;
	}
	else {
		u8 len = 2;
		if(chip->crc_en) {
			len += 1;
		}
		if(chip->burst) {
			len += 2;
			if(!write) {
				len += three_regs_burst ? 3 : 2;
			}
		}
		return len;
	}
}

/*static int __max14916_write(struct max14916_chip *chip, u8 register) {
	int buffer_len = __max14916_get_data_length(chip, true, );
	if(chip->mode == ADDRESS_BITS) {

	}
	else {

	}

	return spi_write_then_read(to_spi_device(chip->gpio_chip.parent), chip->buffer, 1);
}
*/

static int max14916_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct max14916_chip *chip = gpiochip_get_data(gc);
	u8 bank = chip->ndaisy_chained - 1 - offset / 8;
	u8 pin = offset % 8;
	int ret;

	mutex_lock(&chip->lock);
	ret = (chip->pin_buffer[bank] >> pin) & 0x1;
	mutex_unlock(&chip->lock);

	return ret;
}

static void max14916_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
	struct max14916_chip *chip = gpiochip_get_data(gc);
	struct spi_transfer transfer = {};
	u8 len;
	u8 bank = chip->ndaisy_chained - 1 - offset / 8;
	u8 pin = offset % 8;
	u8 i;
	chip->burst = false;
	mutex_lock(&chip->lock);

	if (chip->mode == ADDRESS_BITS) {
		if (val)
			chip->pin_buffer[bank] |= (1 << pin);
		else
			chip->pin_buffer[bank] &= ~(1 << pin);

		len = __max14916_get_data_length(chip, true, false);

		u8 txBuffer[len];
		u8 rxBuffer[len];
		memset(txBuffer, 0x0, sizeof(txBuffer));

		txBuffer[0] = ((0b00001111 & MAX14916_REG_SET_OUT) << 1) | chip->address_bit_1 << 7 | chip->address_bit_0 << 6 | chip->burst << 5 | 1 << 0;
		txBuffer[1] = chip->pin_buffer[bank];

		memset(rxBuffer, 0x0, sizeof(rxBuffer));
		transfer.tx_buf = txBuffer;
		transfer.rx_buf = rxBuffer;
		transfer.len = len;

		int temp = spi_sync_transfer(to_spi_device(chip->gpio_chip.parent), &transfer, 1);

		if(temp) {
			//error
			dev_err(chip->gpio_chip.parent, "spi_write_then_read ERROR %d", temp);
		}
		else {
			//handle rx
			//filter HiZ
			rxBuffer[0] &= 0b00111111;
			if(rxBuffer[0]) {
				//error
				int globlF = (rxBuffer[0] & (1 << 0)) > 0;
				int overLdF = (rxBuffer[0] & (1 << 1)) > 0;
				int currLim = (rxBuffer[0] & (1 << 2)) > 0;
				int owOffF = (rxBuffer[0] & (1 << 3)) > 0;
				int owOnF = (rxBuffer[0] & (1 << 4)) > 0;
				int shrtVDD = (rxBuffer[0] & (1 << 5)) > 0;

				dev_err(chip->gpio_chip.parent, "Fault flags: GloblF: %d, OverLdF: %d, CurrLim: %d, OWOffF: %d, OWOnF: %d, ShrtVDD: %d",
					globlF, overLdF, currLim, owOffF, owOnF, shrtVDD);

				if (globlF) {
					dev_err(chip->gpio_chip.parent, "reading/clearing GloblF");
					memset(txBuffer, 0x0, sizeof(txBuffer));
					txBuffer[0] = ((0b00001111 & MAX14916_REG_GLOBAL_ERR) << 1) | chip->address_bit_1 << 7 | chip->address_bit_0 << 6 | chip->burst << 5 | 0 << 0;
					memset(rxBuffer, 0x0, sizeof(rxBuffer));
					transfer.tx_buf = txBuffer;
					transfer.rx_buf = rxBuffer;
					transfer.len = len;
					int temp = spi_sync_transfer(to_spi_device(chip->gpio_chip.parent), &transfer, 1);

					if(temp) {
						//error
						dev_err(chip->gpio_chip.parent, "spi_write_then_read ERROR %d", temp);
					}
					else if(rxBuffer[1]) {
						//error
						int vint_UV = (rxBuffer[1] & (1 << 0)) > 0;
						int vA_UVLO = (rxBuffer[1] & (1 << 1)) > 0;
						int vddNotGood = (rxBuffer[1] & (1 << 2)) > 0;
						int vddWarn = (rxBuffer[1] & (1 << 3)) > 0;
						int vddUvlo = (rxBuffer[1] & (1 << 4)) > 0;
						int thrmShutd = (rxBuffer[1] & (1 << 5)) > 0;
						int synchErr = (rxBuffer[1] & (1 << 6)) > 0;
						int wdErr = (rxBuffer[1] & (1 << 7)) > 0;
						dev_err(chip->gpio_chip.parent, "Global errors: Vint_UV: %d, VA_UVLO: %d, VddNotGood: %d, VddWarn: %d, VddUvlo: %d, ThrmShutd: %d, SynchErr: %d, WDErr: %d",
							vint_UV, vA_UVLO, vddNotGood, vddWarn, vddUvlo, thrmShutd, synchErr, wdErr);
					}
				}
			}
			if(rxBuffer[1]) {
				//channel error
				for(i = 0; i < (sizeof(rxBuffer[1]) * 8); i++) {
					if(rxBuffer[1] & (1 << i)) {
						dev_err(chip->gpio_chip.parent, "Fault on Channel %d: ", i+1);
					}
				}
			}
		}

	}
	mutex_unlock(&chip->lock);
}
static void max14916_set_multiple(struct gpio_chip *gc, unsigned long *mask,
				    unsigned long *bits)
{
	struct max14916_chip *chip = gpiochip_get_data(gc);
	unsigned int i, idx, shift;
	u8 bank, bankmask;

	mutex_lock(&chip->lock);
	for (i = 0, bank = chip->ndaisy_chained - 1; i < chip->ndaisy_chained;
	     i++, bank--) {
		idx = i / sizeof(*mask);
		shift = i % sizeof(*mask) * BITS_PER_BYTE;
		bankmask = mask[idx] >> shift;
		if (!bankmask)
			continue;

		chip->pin_buffer[bank] &= ~bankmask;
		chip->pin_buffer[bank] |= bankmask & (bits[idx] >> shift);
	}
	//__max14916_write_config();
	mutex_unlock(&chip->lock);
}

static int max14916_direction_output(struct gpio_chip *gc,
		unsigned offset, int val)
{
	max14916_set_value(gc, offset, val);
	return 0;
}
static int max14916_probe(struct spi_device *spi)
{
	struct max14916_chip *chip;
	u32 nregs = 1;
	int ret;
	u32 abits = 0;
	bool crc_en = false;
	enum max14916_mode mode = DAISY_CHAIN;

	spi->bits_per_word = 8;
	spi->mode = 0;
	spi->max_speed_hz = 10000000;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	mode = device_property_read_bool(&spi->dev, MAX14916_DT_PROP_AMODE);

	crc_en = device_property_read_bool(&spi->dev, MAX14916_DT_PROP_CRC);

	//addressed mode
	if(mode) {
		if(device_property_present(&spi->dev, MAX14916_DT_PROP_ADDRESS)) {
			if(device_property_read_u32(&spi->dev, MAX14916_DT_PROP_ADDRESS, &abits) || abits > 3) {
				return -EINVAL;
			}
		}
		else {
			return -EINVAL;
		}
	} else {
		if(device_property_present(&spi->dev, MAX14916_DT_PROP_DAISY)) {
			device_property_read_u32(&spi->dev, MAX14916_DT_PROP_DAISY, &nregs);
			if(nregs == 0) {
				return -EINVAL;
			}
		}
		else {
			return -EINVAL;
		}
	}


	chip = devm_kzalloc(&spi->dev, sizeof(*chip) + nregs, GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->mode = mode;
	chip->ndaisy_chained = nregs;

	if(chip->mode == ADDRESS_BITS) {
		chip->address_bit_0 = (abits & 1) > 0;
		chip->address_bit_1 = (abits & (1 << 1)) > 0;
	}

	chip->burst = false; // No burst implementation (yet)

	chip->gpiod_oe = devm_gpiod_get_optional(&spi->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(chip->gpiod_oe))
		return PTR_ERR(chip->gpiod_oe);

	gpiod_set_value_cansleep(chip->gpiod_oe, 1);

	spi_set_drvdata(spi, chip);

	chip->gpio_chip.label = spi->modalias;
	chip->gpio_chip.direction_output = max14916_direction_output;
	chip->gpio_chip.get = max14916_get_value;
	chip->gpio_chip.set = max14916_set_value;
	chip->gpio_chip.base = -1;

	memset(chip->pin_buffer, 0, chip->ndaisy_chained);


	chip->gpio_chip.ngpio = MAX14916_NUMBER_GPIOS * chip->ndaisy_chained;

	chip->gpio_chip.can_sleep = true;
	chip->gpio_chip.parent = &spi->dev;
	chip->gpio_chip.owner = THIS_MODULE;

	mutex_init(&chip->lock);

	/*ret = __max14916_write_config(chip);
	if (ret) {
		dev_err(&spi->dev, "Failed writing: %d\n", ret);
		goto exit_destroy;
	}
	*/
	ret = gpiochip_add_data(&chip->gpio_chip, chip);
	if (!ret)
		return 0;

exit_destroy:
	mutex_destroy(&chip->lock);

	return ret;
}

static void max14916_remove(struct spi_device *spi)
{
	struct max14916_chip *chip = spi_get_drvdata(spi);

	gpiod_set_value_cansleep(chip->gpiod_oe, 0);
	gpiochip_remove(&chip->gpio_chip);
	mutex_destroy(&chip->lock);

}

#ifdef CONFIG_OF
static const struct of_device_id max14916_of_id[] = {
	{ .compatible = "maxim,max14916" },
	{ }
};
MODULE_DEVICE_TABLE(of, max14916_of_id);
#endif

static const struct spi_device_id max14916_spi_id[] = {
	{ "max14916" },
	{ }
};
MODULE_DEVICE_TABLE(spi, max14916_spi_id);

static struct spi_driver max14916_driver = {
	.driver = {
		.name		= "max14916",
		.of_match_table	= of_match_ptr(max14916_of_id),
	},
	.probe	  = max14916_probe,
	.remove	  = max14916_remove,
	.id_table = max14916_spi_id,
};


module_spi_driver(max14916_driver);

MODULE_AUTHOR("Philipp Harms <philipp.harms@i-graphx.com>");
MODULE_DESCRIPTION("GPIO expander driver for MAX14916 8 Output Power IC");
MODULE_LICENSE("GPL v2");
