#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/pm.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>

#include <asm/unaligned.h>

#define GSLX680_TS_NAME        "gslx680_ts"

#define SILEAD_REG_RESET       0xE0
#define SILEAD_REG_DATA        0x80
#define SILEAD_REG_TOUCH_NR    0x80
#define SILEAD_REG_POWER       0xBC
#define SILEAD_REG_POWER2      0XBD
#define SILEAD_REG_POWER3      0XBE
#define SILEAD_REG_POWER4      0xBF
#define SILEAD_REG_CLOCK       0xE4
#define SILEAD_REG_STATUS      0xB0
#define SILEAD_REG_ID          0xFC
#define SILEAD_REG_MEM_CHECK   0xB0

#define SILEAD_STATUS_OK       0x5A5A5A5A
#define SILEAD_TS_DATA_LEN     44
#define SILEAD_CLOCK           0x04

#define SILEAD_CMD_RESET       0x88
#define SILEAD_CMD_START       0x00

#define SILEAD_MAX_X           4095
#define SILEAD_MAX_Y           4095

#define SILEAD_POINT_DATA_LEN  0x04
#define SILEAD_POINT_Y_OFF     0x00
#define SILEAD_POINT_Y_MSB_OFF 0x01
#define SILEAD_POINT_X_OFF     0x02
#define SILEAD_POINT_X_MSB_OFF 0x03
#define SILEAD_EXTRA_DATA_MASK 0xF0
#define SILEAD_POINT_HSB_MASK  0x0F
#define SILEAD_TOUCH_ID_MASK   0xF0

#define SILEAD_CMD_SLEEP_MIN   10000
#define SILEAD_CMD_SLEEP_MAX   20000
#define SILEAD_POWER_SLEEP     20
#define SILEAD_STARTUP_SLEEP   30

#define MAX_FINGERS            10

enum gslx680_ts_power {
	GSLX680_POWER_ON = 1,
	GSLX680_POWER_OFF = 0
};

struct gslx680_ts_data {
	struct i2c_client *client;
	struct gpio_desc *gpio_power;
	struct input_dev *input;
	struct regulator_bulk_data regulators[2];
	char fw_name[64];
	u32 x_max;
	u32 y_max;
	bool x_invert;
	bool y_invert;
	bool xy_swap;
	struct touchscreen_properties prop;
	u32 max_fingers;
	u32 chip_id;
	struct input_mt_pos pos [MAX_FINGERS];
	int slots [MAX_FINGERS];
	int id[MAX_FINGERS];
};

struct gslx680_fw_data {
	u32 offset;
	u32 val;
};

static int gslx_ts_request_input_dev(struct gslx680_ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	data->input = devm_input_allocate_device(dev);
	if (!data->input) {
		dev_err(dev,
			"failed to allocate input device\n");
		return -ENOMEM;
	}

	input_set_abs_params(data->input, ABS_MT_POSITION_X, 0,
			     data->x_max, 0, 0);
	input_set_abs_params(data->input, ABS_MT_POSITION_Y, 0,
			     data->y_max, 0, 0);
	touchscreen_parse_properties(data->input, true, &data->prop);
	/*	printk(KERN_INFO "I got x=%d", data->prop->);*/

	input_mt_init_slots(data->input, data->max_fingers,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED |
			    INPUT_MT_TRACK);

	data->input->name = GSLX680_TS_NAME;
	data->input->phys = "input/ts";
	data->input->id.bustype = BUS_I2C;

	error = input_register_device(data->input);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}



static int gslx680_ts_set_default_fw_name(struct gslx680_ts_data *data,
					  const struct i2c_device_id *id)
{
	snprintf(data->fw_name, sizeof(data->fw_name),
		 "silead/%s.fw", id->name);
	printk(KERN_INFO "Oopsie....\n");
	return 0;
}

static void gslx680_ts_read_props(struct i2c_client *client)
{
	struct gslx680_ts_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	const char *str;
	int error;

	error = device_property_read_u32(dev, "silead,max-fingers",
					 &data->max_fingers);

	if (error) {
		dev_dbg(dev, "Max fingers read error %d\n", error);
		data->max_fingers = 5; /* Make it 5 fingers*/
	}

	error = device_property_read_string(dev, "firmware-name", &str);
	if (!error) 
		snprintf(data->fw_name, sizeof(data->fw_name),
			 "silead/%s", str);
	else
		dev_dbg(dev, "Firmware file name read error. Using default.");

	error = device_property_read_u32(dev, "touchscreen-size-x", &data->x_max);
	if (error)
	{
		dev_dbg(dev, "Resolution in X read error %d\n", error);
		data->x_max = SILEAD_MAX_X;
	}

	error = device_property_read_u32(dev, "touchscreen-size-y", &data->y_max);
	if (error)
	{
		dev_dbg(dev, "Resolution in Y read error %d\n", error);
		data->y_max = SILEAD_MAX_Y;
	}

	data->x_invert = device_property_read_bool(dev, "touchscreen-inverted-x");
	data->y_invert = device_property_read_bool(dev, "touchscreen-inverted-y");
	data->xy_swap = device_property_read_bool(dev, "touchscreen-swapped-x-y");

	printk(KERN_INFO "I read x_max = %d, y_max = %d, max_fingers = %d, x_inv = %d, y_inv = %d, xy_swap = %d\n", data->x_max, data->y_max, data->max_fingers, data->x_invert,
	       data->y_invert, data->xy_swap);
	
}

static void gslx680_ts_report_touch(struct gslx680_ts_data *data, u16 x, u16 y, u8 id)
{
	if (data->x_invert)
		x = data->x_max - x;

	if (data->y_invert)
		y = data->y_max - y;

	if (data->xy_swap)
		swap(x, y);

	input_mt_slot(data->input, id);
	input_mt_report_slot_state(data->input, MT_TOOL_FINGER, true);
	input_report_abs(data->input, ABS_MT_POSITION_X, x);
	input_report_abs(data->input, ABS_MT_POSITION_Y, y);
}

static void gslx680_ts_read_data(struct i2c_client *client)
{
	struct gslx680_ts_data *data = i2c_get_clientdata(client);
	/*	struct input_dev *input = data->input;*/
	struct device *dev = &client->dev;
	u8 buf[SILEAD_TS_DATA_LEN];
	int touch_nr, x, y, id, offset, index, error, i;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_DATA,
					      SILEAD_TS_DATA_LEN, buf);
	if (error < 0) {
		dev_err(dev, "Illegal finger number detection. Data read error %d\n", error);
		return;
	}

	if (buf[0] > data->max_fingers) {
		dev_warn(dev, "More touches reported than supported %d > %d\n",
			 buf[0], data->max_fingers);
		buf[0] = data->max_fingers;
	}

	touch_nr = buf[0];

	if (touch_nr < 0)
		return;

	dev_dbg(dev, "Touch number: %d\n", touch_nr);

	for (i = 1; i <= touch_nr; i++) {
		offset = i * SILEAD_POINT_DATA_LEN;

		/* Bits 4-7 are the touch id */
		id = (buf[offset + SILEAD_POINT_X_MSB_OFF] &
		      SILEAD_TOUCH_ID_MASK) >> 4;

		/* Bits 0-3 are MSB of X */
		buf[offset + SILEAD_POINT_X_MSB_OFF] =
			buf[offset + SILEAD_POINT_X_MSB_OFF] &
			SILEAD_POINT_HSB_MASK;

		/* Bits 0-3 are MSB of Y */
		buf[offset + SILEAD_POINT_Y_MSB_OFF] =
			buf[offset + SILEAD_POINT_Y_MSB_OFF] &
			SILEAD_POINT_HSB_MASK;

		y = le16_to_cpup((__le16 *)(buf + offset + SILEAD_POINT_Y_OFF));
		x = le16_to_cpup((__le16 *)(buf + offset + SILEAD_POINT_X_OFF));

		index = i - 1;
		data->pos[index].x = x;
		data->pos[index].y = y;

		input_mt_assign_slots(data->input, data->slots, data->pos,
				      index, 0);
		gslx680_ts_report_touch(data, x, y, data->slots[index]);

		dev_dbg(dev, "x=%d y=%d hw_id=%d sw_id=%d\n", x, y, id,
			data->slots[index]);
	}

	input_sync(data->input);

}

static irqreturn_t gslx680_ts_threaded_irq_handler(int irq, void *id)
{
	struct gslx680_ts_data *data = id;
	struct i2c_client *client = data->client;

	gslx680_ts_read_data(client);

	return IRQ_HANDLED;
}

static void gslx680_disable_regulator(void *arg)
{
	struct gslx680_ts_data *data = arg;

	regulator_bulk_disable(ARRAY_SIZE(data->regulators), data->regulators);
}

static void gslx680_ts_set_power(struct i2c_client *client,
				 enum gslx680_ts_power state)
{
	struct gslx680_ts_data *data = i2c_get_clientdata(client);

	if (data->gpio_power) {
		gpiod_set_value_cansleep(data->gpio_power, state);
		msleep(SILEAD_POWER_SLEEP);
	}
}

static int gslx680_ts_reset(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					  SILEAD_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK,
					  SILEAD_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER, SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER2, SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER3, SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER4, SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Chip reset error %d\n", error);
	return error;
}

static int gslx680_ts_get_id(struct i2c_client *client)
{
	struct gslx680_ts_data *data = i2c_get_clientdata(client);
	__le32 chip_id;
	int error;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_ID,
					      sizeof(chip_id), (u8 *)&chip_id);
	if (error < 0) {
		dev_err(&client->dev, "Chip ID read error %d\n", error);
		return error;
	}

	data->chip_id = le32_to_cpu(chip_id);
	dev_info(&client->dev, "Silead chip ID 0x%8X", data->chip_id);

	return 0;
}

static u32 gslx680_ts_get_status(struct i2c_client *client)
{
	int error;
	__le32 status;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_STATUS,
					      sizeof(status), (u8 *)&status);
	if (error < 0) {
		dev_err(&client->dev, "Status read error %d\n", error);
		return error;
	}

	return le32_to_cpu(status);
}

static int gslx680_ts_startup(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET, 0x00);
	if (error) {
		dev_err(&client->dev, "Startup error %d\n", error);
		return error;
	}
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;
}

static int gslx680_ts_load_fw(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gslx680_ts_data *data = i2c_get_clientdata(client);
	unsigned int fw_size, i;
	const struct firmware *fw;
	struct gslx680_fw_data *fw_data;
	int error;

	dev_dbg(dev, "Firmware file name: %s", data->fw_name);

	error = request_firmware(&fw, data->fw_name, dev);
	if (error) {
		dev_err(dev, "Firmware request error %d\n", error);
		return error;
	}

	fw_size = fw->size / sizeof(*fw_data);
	fw_data = (struct gslx680_fw_data *)fw->data;

	for (i = 0; i < fw_size; i++) {
		error = i2c_smbus_write_i2c_block_data(client,
						       fw_data[i].offset,
						       4,
						       (u8 *)&fw_data[i].val);
		if (error) {
			dev_err(dev, "Firmware load error %d\n", error);
			break;
		}
	}

	release_firmware(fw);
	return error ?: 0;
}

static int gslx680_ts_setup(struct i2c_client *client)
{
	int error;
	u32 status;

	gslx680_ts_set_power(client, GSLX680_POWER_OFF);
	gslx680_ts_set_power(client, GSLX680_POWER_ON);

	error = gslx680_ts_get_id(client);
	if (error)
		return error;

	gslx680_ts_set_power(client, GSLX680_POWER_OFF);
	gslx680_ts_set_power(client, GSLX680_POWER_ON);

	error = gslx680_ts_reset(client);
	if (error)
		return error;

	error = gslx680_ts_load_fw(client);
	if (error)
		return error;

	error = gslx680_ts_startup(client);
	if (error)
		return error;
	
	error = gslx680_ts_reset(client);
	if (error)
		return error;

	gslx680_ts_set_power(client, GSLX680_POWER_OFF);
	usleep_range(50000, 55000);
	gslx680_ts_set_power(client, GSLX680_POWER_ON);
	usleep_range(30000, 35000);
	gslx680_ts_set_power(client, GSLX680_POWER_OFF);
	usleep_range(5000, 6000);
	gslx680_ts_set_power(client, GSLX680_POWER_ON);
	usleep_range(20000, 22000);

	error = gslx680_ts_reset(client);
	if (error)
		return error;

	error = gslx680_ts_startup(client);
	if (error)
		return error;

	status = gslx680_ts_get_status(client);
	if (status != SILEAD_STATUS_OK) {
		dev_err(&client->dev,
			"Initialization error, status: 0x%X\n", status);
		return -ENODEV;
	}

	printk(KERN_INFO "Hooray!");
	
	return 0;
}

static int gslx680_ts_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct gslx680_ts_data *data;
	struct device *dev = &client->dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		dev_err(dev, "I2C functionality check failed\n");  
		printk(KERN_INFO "I f'd a bit.\n"); /* Esto no pasa */
		return -ENXIO;
	}

	printk(KERN_INFO "Functionality checked! ^_^\n");
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	error = gslx680_ts_set_default_fw_name(data, id);
	if (error)
		return error;

	printk(KERN_INFO "past set_default_fw_name\n");

	gslx680_ts_read_props(client);
	
	/* We must have the IRQ provided by DT */
	if (client->irq <= 0) {
	  	printk(KERN_INFO "Got lost past this point!\n"); /* Esto no pasa */
		return -ENODEV;
	}
		
	data->regulators[0].supply = "vddio";
	data->regulators[1].supply = "avdd";
	error = devm_regulator_bulk_get(dev, ARRAY_SIZE(data->regulators),
					data->regulators);
	if (error) {
		printk(KERN_INFO "Pretty sure this shouldn't have happened\n"); /* Esto no pasa */
		return error;
	}

	/* Enable regulators at probe and disable them at remove, we need
	   to keep the chip powered otherwise it forgets its firmware. */
	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators),
				      data->regulators);
	if (error) {
		printk(KERN_INFO "Dunno what went on here\n"); /* Esto no pasa */
		return error;
	}

	error = devm_add_action_or_reset(dev, gslx680_disable_regulator, data);
	if (error) {
		printk(KERN_INFO "This should not happen."); /* Esto no pasa */
		return error;
	}
	
	/* Power GPIO pin */
	data->gpio_power = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(data->gpio_power)) {
		if (PTR_ERR(data->gpio_power) != -EPROBE_DEFER)
			dev_err(dev, "Shutdown GPIO request failed.\n"); /* Esto no pasa */
		printk(KERN_INFO "Farted here =/"); /* Esto no pasa */
		return PTR_ERR(data->gpio_power);
	}

	printk(KERN_INFO "Cleared GPIO part already!\n");

	error = gslx680_ts_setup(client);
	if (error)
		return error;

	error = gslx_ts_request_input_dev(data);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, client->irq,
					  NULL,
					  gslx680_ts_threaded_irq_handler,
					  IRQF_ONESHOT, client->name, data);
	if (error) {
		if (error != -EPROBE_DEFER)
			dev_err(dev, "IRQ request failed %d\n", error);
		return error;
	}
	
	return 0;

}



static const struct i2c_device_id gslx680_ts_id[] = {
	{ "gsl1680", 0 },
	{ "gsl1688", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, gslx680_ts_id);

static const struct of_device_id gslx680_ts_of_match[] = {
	{ .compatible = "silead,gsl1680" },
	{ .compatible = "silead,gsl1688" },
	{ },
};
MODULE_DEVICE_TABLE(of, gslx680_ts_of_match);

static struct i2c_driver gslx680_ts_driver = {
	.probe = gslx680_ts_probe,
	.id_table = gslx680_ts_id,
	.driver = {
		.name = GSLX680_TS_NAME,
		.of_match_table = of_match_ptr(gslx680_ts_of_match),
	/*	.pm = &gslx680_ts_pm,*/
	},	
};
module_i2c_driver(gslx680_ts_driver);

MODULE_AUTHOR("Robert Dolca");
MODULE_DESCRIPTION("GSLX680 Silead i2c touchscreen driver");
MODULE_LICENSE("GPL");
