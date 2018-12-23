#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/spinlock.h>

/*----------------------------------------------------------------------------*/
/*
/dts-v1/;
/plugin/;
/ {
	compatible = "allwinner,sun8i-h3";

	fragment@0 {
		target = <&i2c0>;
		__overlay__ {
			#address-cells = <1>;
            #size-cells = <0>;
            status = "okay";

            mcp23016@20 {
                    compatible = "microchip,mcp23016";
                    reg = <0x20>;
                    status = "okay";
            };
		};
		__fixups__ {
			i2c0 = "/fragment@0:target:0";
		};
	};
};
*/
/*----------------------------------------------------------------------------*/
#define GPIO_NUM 16
#define INPUT 1
#define OUTPUT 0

/* Chip Control registers */

/*
 * GP0 ang GP1 provides access to GPIO ports
 * A read from these register provide status of pin of these ports
 * A write to these register will modify the output latch register (OLAT0, OLAT1)
 * and data register.
 */
#define GP0		0x0
#define GP1		0x1

/*
 * IOLAT0 and IOLAT1 control the output value of GP0 and GP1
 * Write into one of these register will result in affecting
 */
// Фиксируются биты при записи в порт
#define OLAT0	0x2
#define OLAT1	0x3

// Регистры выбора полярности
#define IPOL0	0x4
#define IPOL1	0x5

/*
 * IODIR0 and IODIR1 registers control GP0 and GP1 IOs direction
 * 1 => input; 0 => output
 * default value are 0xFF in each reg.
 */
#define IODIR0	0x6
#define IODIR1	0x7

/*
 * INTCAP0 and INTCAP1 register contain value of the port that generated the interupt
 * INTCAP0 contains value of GPO at time of GPO change interrupt
 * INTCAP1 contains value of GP1 at time of GP1 change interrupt
 */
#define INTCAP0	0x8
#define INTCAP1	0x9

#define IOCON0 0xA
#define IOCON1 0xB

#define MS_TO_NS(x) (x * 1E6L)

/*----------------------------------------------------------------------------*/
//struct workqueue_struct *wq;
struct mcp23016 {
	struct i2c_client *client;
	struct gpio_chip chip;
	struct hrtimer hr_timer;
	struct work_struct work;
	unsigned irq_base;
};

static struct gpio_desc *led;
static unsigned int irq;

/*----------------------------------------------------------------------------*/
static inline struct mcp23016 *to_mcp23016(struct gpio_chip *gc)
{
	return container_of(gc, struct mcp23016, chip);
}
/*----------------------------------------------------------------------------*/
/*static void synaptics_ts_work_func(struct work_struct *work)
{
	s32 iodirval;
	struct mcp23016 *mcp = container_of(work, struct mcp23016, work);

	pr_info("MCP23016: -------------------------------------------\n");

	// Вычитываем состояние регистров
	iodirval = i2c_smbus_read_byte_data(mcp->client, GP0);
	pr_info("MCP23016: GP0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, GP1);
	pr_info("MCP23016: GP1: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, OLAT0);
	pr_info("MCP23016: OLAT0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, OLAT1);
	pr_info("MCP23016: OLAT1: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IPOL0);
	pr_info("MCP23016: IPOL0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IPOL1);
	pr_info("MCP23016: IPOL1: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IODIR0);
	pr_info("MCP23016: IODIR0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IODIR1);
	pr_info("MCP23016: IODIR1: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP0);
	pr_info("MCP23016: INTCAP0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP1);
	pr_info("MCP23016: INTCAP1: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IOCON0);
	pr_info("MCP23016: IOCON0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, IOCON1);
	pr_info("MCP23016: IOCON1: 0x%x\n", iodirval);
}*/
/*----------------------------------------------------------------------------*/
/*static enum hrtimer_restart my_hrtimer_callback(struct hrtimer *timer)
{
	ktime_t currtime , interval;
	struct mcp23016 *mcp = container_of(timer, struct mcp23016, hr_timer);

	//pr_info("MCP23016: Timer Handler called.\n");

	currtime  = ktime_get();
	interval = ktime_set(0, MS_TO_NS(5000));
	hrtimer_forward(timer, currtime , interval);
	//pr_info("My_hrtimer_callback called (%ld).\n", jiffies);

	queue_work(wq, &mcp->work);

	return HRTIMER_RESTART;
}*/
/*----------------------------------------------------------------------------*/
// Вызывается когда разрешается прерывание
void mcp23016_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);
	s32 iodirval;

	pr_info("MCP23016: hwirq - %d\n", data->hwirq);
	pr_info("MCP23016: irq_base - %d\n", mcp->chip.base);
	//pr_info("MCP23016: client - 0x%x\n", mcp->client);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP0);
	pr_info("MCP23016: INTCAP0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP1);
	pr_info("MCP23016: INTCAP1: 0x%x\n", iodirval);
	pr_info("MCP23016: mcp23016_gpio_irq_unmask\n");
}
/*----------------------------------------------------------------------------*/
// Вызывается когда запрещается прерывание
void mcp23016_gpio_irq_mask(struct irq_data *data)
{
	pr_info("MCP23016: mcp23016_gpio_irq_mask\n");
}
/*----------------------------------------------------------------------------*/
// Вызывается когда вводится тип прерывания - rising, falling, both
int	mcp23016_gpio_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	pr_info("MCP23016: flow_type - %d\n", flow_type);
	return 0; // Присвоение типа прерывания удачно
}
/*----------------------------------------------------------------------------*/
static struct irq_chip mcp23016_irq_chip = {
	.name = "mcp23016-gpio",
	.irq_mask = mcp23016_gpio_irq_mask,
	.irq_unmask = mcp23016_gpio_irq_unmask,
	.irq_set_type = mcp23016_gpio_irq_set_type,
};
/*----------------------------------------------------------------------------*/
static irqreturn_t mcp23016_irq(int irq, void *data)
{
	//struct mcp23016 *mcp = data;
	//unsigned int child_irq, i;
	//s32 iodirval;

	//pr_info("MCP23016: client - 0x%x\n", mcp->client);
	//iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP0);
	//pr_info("MCP23016: INTCAP0: 0x%x\n", iodirval);
	//iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP1);
	//pr_info("MCP23016: INTCAP1: 0x%x\n", iodirval);



	/*for (i = 0; i < mcp->chip.ngpio; i++) {
		if (gpio_value_changed_and_raised_irq(i)) {
			child_irq = irq_find_mapping(mcp->chip.irqdomain, i);
			handle_nested_irq(child_irq);
		}
	}*/

	return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------*/
static int mcp23016_to_irq(struct gpio_chip *chip, unsigned offset)
{
	pr_info("MCP23016: mcp23016_to_irq, offset - %d\n", offset);
	return irq_create_mapping(chip->irqdomain, offset);
}
/*----------------------------------------------------------------------------*/
static int mcp23016_get_value(struct gpio_chip *gc, unsigned offset)
{
	s32 value;
	struct mcp23016 *mcp = to_mcp23016(gc);
	unsigned bank = offset / 8 ;
	unsigned bit = offset % 8 ;

	u8 reg_intcap = (bank == 0) ? INTCAP0 : INTCAP1;
	value = i2c_smbus_read_byte_data(mcp->client, reg_intcap);

	return (value >= 0) ? (value >> bit) & 0x1 : 0;
}
/*----------------------------------------------------------------------------*/
static int mcp23016_set(struct mcp23016 *mcp, unsigned offset, int val)
{
	s32 value;

	unsigned bank = offset / 8 ;
	u8 reg_gpio = (bank == 0) ? GP0 : GP1;
	unsigned bit = offset % 8 ;

	value = i2c_smbus_read_byte_data(mcp->client, reg_gpio);
	if (value >= 0) {
		if (val)
			value |= 1 << bit;
		else
			value &= ~(1 << bit);

		return i2c_smbus_write_byte_data(mcp->client, reg_gpio, value);
	}

	return value;
}
/*----------------------------------------------------------------------------*/
static void mcp23016_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
	struct mcp23016 *mcp = to_mcp23016(gc);
	mcp23016_set(mcp, offset, val);
}
/*----------------------------------------------------------------------------*/
/*
 * direction = 1 => input
 * direction = 0 => output
 */
static int mcp23016_direction(struct gpio_chip *gc, unsigned offset,
                                unsigned direction, int val)
{
	struct mcp23016 *mcp = to_mcp23016(gc);
	// Select port A or B
	// offset = 0 to 15
	unsigned bank = offset / 8 ;
	unsigned bit = offset % 8 ;
	u8 reg_iodir = (bank == 0) ? IODIR0 : IODIR1;
	s32 iodirval = i2c_smbus_read_byte_data(mcp->client, reg_iodir);

	if (direction)
		iodirval |= 1 << bit;
	else
		iodirval &= ~(1 << bit);

	i2c_smbus_write_byte_data(mcp->client, reg_iodir, iodirval);

	if (direction)
		return iodirval ;
	else
		return mcp23016_set(mcp, offset, val);
}
/*----------------------------------------------------------------------------*/
static int mcp23016_direction_output(struct gpio_chip *gc,
                                    unsigned offset, int val)
{
	pr_info("MCP23016: Direction output\n");
	return mcp23016_direction(gc, offset, OUTPUT, val);
}
/*----------------------------------------------------------------------------*/
static int mcp23016_direction_input(struct gpio_chip *gc,
                                    unsigned offset)
{
	pr_info("MCP23016: Direction input\n");
	return mcp23016_direction(gc, offset, INPUT, 0);
}
/*----------------------------------------------------------------------------*/
static const struct of_device_id mcp23016_ids[] = {
	{ .compatible = "microchip,mcp23016", },
	{ /* sentinel */ }
};
/*----------------------------------------------------------------------------*/
static int mcp23016_irq_domain_map(struct irq_domain *domain, unsigned int virq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq,
							&mcp23016_irq_chip,
							handle_level_irq); /* Level trigerred irq */
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct irq_domain_ops mcp23016_irq_domain_ops = {
		.map = mcp23016_irq_domain_map,
		.xlate = irq_domain_xlate_onetwocell,
};
/*----------------------------------------------------------------------------*/
static int mcp23016_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	struct mcp23016 *mcp;
	struct device *dev = &client->dev;
	int retval;
	struct platform_device *pdev = to_platform_device(dev);
	//struct st_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	//ktime_t ktime;

	pr_info("MCP23016: Starting module\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	mcp = devm_kzalloc(&client->dev, sizeof(*mcp), GFP_KERNEL);
	if (!mcp)
		return -ENOMEM;

	led = gpiod_get(dev, "led", GPIOD_OUT_HIGH);
	retval = gpiod_direction_output(led, 0);
	if(retval)
	{
		pr_err("MCP23016 Error led <gpiod_direction_output> \n");
		return -1;
	}

	mcp->chip.label = client->name;
	mcp->chip.base = -1;
	mcp->chip.parent = &client->dev;
	mcp->chip.owner = THIS_MODULE;
	mcp->chip.ngpio = GPIO_NUM;
	mcp->chip.can_sleep = 1;
	mcp->chip.get = mcp23016_get_value;
	mcp->chip.set = mcp23016_set_value;
	mcp->chip.direction_output = mcp23016_direction_output;
	mcp->chip.direction_input = mcp23016_direction_input;
	mcp->chip.to_irq = mcp23016_to_irq;
	mcp->chip.irqdomain = irq_domain_add_linear(client->dev.of_node,
												mcp->chip.ngpio,
												&mcp23016_irq_domain_ops,
												mcp);
	mcp->client = client;
	//pr_info("MCP23016: client - 0x%x\n", client);

	//INIT_WORK(&mcp->work, synaptics_ts_work_func);
	//wq = create_singlethread_workqueue("cmp23016_wq");
	//if(!wq) return -EBADRQC;

	client->irq = platform_get_irq(pdev, 0);

	i2c_set_clientdata(client, mcp);

	/* Do we have an interrupt line? Enable the irqchip */
	if (client->irq) {
		retval = gpiochip_irqchip_add(&mcp->chip,
										&mcp23016_irq_chip,
										0, handle_level_irq, IRQ_TYPE_NONE);
		if (retval) {
			dev_err(&client->dev, "cannot add irqchip\n");
			goto fail_irq;
		}
		retval = devm_request_threaded_irq(&client->dev, client->irq, NULL,
											mcp23016_irq, IRQF_ONESHOT | IRQF_TRIGGER_FALLING | IRQF_SHARED,
											dev_name(&client->dev), mcp);
		if (retval) {
			goto fail_irq;
		}
		gpiochip_set_chained_irqchip(&mcp->chip,
									&mcp23016_irq_chip,
									client->irq, NULL);

		pr_info("MCP23016: irq - %d\n", client->irq);
	}

	i2c_smbus_write_byte_data(mcp->client, IODIR0, 0x00);
	i2c_smbus_write_byte_data(mcp->client, IODIR1, 0x00);

	i2c_smbus_write_byte_data(mcp->client, GP0, 0xFF);
	i2c_smbus_write_byte_data(mcp->client, GP1, 0xFF);
	mdelay(100);
	i2c_smbus_write_byte_data(mcp->client, GP0, 0x00);
	i2c_smbus_write_byte_data(mcp->client, GP1, 0x00);
	mdelay(100);
	i2c_smbus_write_byte_data(mcp->client, GP0, 0xFF);
	i2c_smbus_write_byte_data(mcp->client, GP1, 0xFF);
	mdelay(100);
	i2c_smbus_write_byte_data(mcp->client, GP0, 0x00);
	i2c_smbus_write_byte_data(mcp->client, GP1, 0x00);

	/*pr_info("MCP23016: HR Timer module installing\n");
	ktime = ktime_set(0, MS_TO_NS(5000));
	hrtimer_init(&mcp->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mcp->hr_timer.function = &my_hrtimer_callback;
	pr_info("MCP23016: Starting timer to fire in %dms (%lu)\n", 1000, jiffies);
	hrtimer_start(&mcp->hr_timer, ktime, HRTIMER_MODE_REL);*/

	return gpiochip_add(&mcp->chip);
fail_irq:
	return -1;
}
/*----------------------------------------------------------------------------*/
static int mcp23016_remove(struct i2c_client *client)
{
	struct mcp23016 *mcp;
	int ret;
	mcp = i2c_get_clientdata(client);
	gpiochip_remove(&mcp->chip);
	gpiod_put(led);
	synchronize_irq(irq);
	free_irq(irq, NULL);

	i2c_smbus_write_byte_data(mcp->client, IODIR0, 0x00);
	i2c_smbus_write_byte_data(mcp->client, IODIR1, 0x00);
	i2c_smbus_write_byte_data(mcp->client, GP0, 0x00);
	i2c_smbus_write_byte_data(mcp->client, GP1, 0x00);

	ret = hrtimer_cancel(&mcp->hr_timer);
	if(ret) pr_info("The timer was still in use...\n");

	/*if(wq) {
		flush_workqueue(wq);
		destroy_workqueue(wq);
	}*/

	pr_info("MCP23016: Removing module\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mcp23016_id[] = {
	{"mcp23016", 0},
	{},
};
/*----------------------------------------------------------------------------*/
MODULE_DEVICE_TABLE(i2c, mcp23016_id);

static struct i2c_driver mcp23016_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mcp23016",
		.of_match_table = of_match_ptr(mcp23016_ids),
	},
	.probe = mcp23016_probe,
	.remove = mcp23016_remove,
	.id_table = mcp23016_id,
};

module_i2c_driver(mcp23016_i2c_driver);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitry");
