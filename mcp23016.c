#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
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
struct mcp23016 {
	struct i2c_client *client;
	struct gpio_chip chip;
	struct hrtimer hr_timer;
	struct work_struct work;
	u16 irq_enable;
	u16 iodir;
	u16 ioport;
	unsigned int edge[GPIO_NUM];
	struct mutex lock;
	struct mutex irq_lock;
};

static char irqName[20] = "I2C: ";

/*----------------------------------------------------------------------------*/
static inline struct mcp23016 *to_mcp23016(struct gpio_chip *gc)
{
	return container_of(gc, struct mcp23016, chip);
}
/*----------------------------------------------------------------------------*/
// Вызывается когда разрешается прерывание
void mcp23016_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);

	pr_info("MCP23016: mcp23016_gpio_irq_unmask\n");

	pr_info("MCP23016: hwirq - %ld\n", data->hwirq);
	pr_info("MCP23016: irq_base - %d\n", mcp->chip.base);
	pr_info("MCP23016: irq - %d\n", data->irq);

	mcp->irq_enable |= BIT(data->hwirq);
	pr_info("MCP23016: irq_enable = 0x%x\n", mcp->irq_enable);
}
/*----------------------------------------------------------------------------*/
// Вызывается когда запрещается прерывание
void mcp23016_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);

	pr_info("MCP23016: mcp23016_gpio_irq_mask\n");

	mcp->irq_enable &= ~BIT(data->hwirq);
	mcp->edge[data->hwirq] = 0;

	pr_info("MCP23016: irq_enable = 0x%x\n", mcp->irq_enable);
}
/*----------------------------------------------------------------------------*/
// Вызывается когда вводится тип прерывания:
// rising - 1
// falling - 2
// both - 3
int	mcp23016_gpio_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);
	pr_info("MCP23016: mcp23016_gpio_irq_set_type, flow_type - 0x%x\n", flow_type);
	mcp->edge[data->hwirq] = flow_type;
	return 0; // Присвоение типа прерывания удачно
}
/*----------------------------------------------------------------------------*/
static void mcp23016_irq_bus_lock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);
	pr_info("MCP23016: mcp23016_irq_bus_lock\n");
	mutex_lock(&mcp->irq_lock);
}
/*----------------------------------------------------------------------------*/
static void mcp23016_irq_bus_unlock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mcp23016 *mcp = to_mcp23016(gc);
	pr_info("MCP23016: mcp23016_irq_bus_unlock\n");
	mutex_unlock(&mcp->irq_lock);
}
/*----------------------------------------------------------------------------*/
static struct irq_chip mcp23016_irq_chip = {
	.name = "mcp23016-gpio",
	.irq_mask = mcp23016_gpio_irq_mask,
	.irq_unmask = mcp23016_gpio_irq_unmask,
	.irq_set_type = mcp23016_gpio_irq_set_type,
	.irq_bus_lock = mcp23016_irq_bus_lock,
	.irq_bus_sync_unlock = mcp23016_irq_bus_unlock,
};
/*----------------------------------------------------------------------------*/
static irqreturn_t mcp23016_irq(int irq, void *data)
{
	struct mcp23016 *mcp = data;
	unsigned int child_irq;
	unsigned long gpio;
	s32 iodirval;
	u16 irqMask;

	pr_info("MCP23016: ************************************************\n");

	mutex_lock(&mcp->lock);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP0);
	irqMask = (u16)iodirval;
	pr_info("MCP23016: INTCAP0: 0x%x\n", iodirval);
	iodirval = i2c_smbus_read_byte_data(mcp->client, INTCAP1);
	irqMask |= (u16)iodirval << 8;
	pr_info("MCP23016: INTCAP1: 0x%x\n", iodirval);
	pr_info("MCP23016: irqMask: 0x%x\n", irqMask);
	pr_info("MCP23016: irq_enable: 0x%x\n", mcp->irq_enable);
	mutex_unlock(&mcp->lock);

	// Перебираем активные прерывания
	for(gpio=0; gpio<mcp->chip.ngpio; gpio++)
	{
		if(mcp->irq_enable & BIT(gpio))
		{
			//  Проверяем тип прерывания: rising, falling, both
			if( (mcp->edge[gpio] == IRQ_TYPE_EDGE_BOTH) ||
				( (mcp->edge[gpio] == IRQ_TYPE_EDGE_RISING) && (irqMask & BIT(gpio)) ) ||
				( (mcp->edge[gpio] == IRQ_TYPE_EDGE_FALLING) && (!(irqMask & BIT(gpio))) )
			) {
				child_irq = irq_find_mapping(mcp->chip.irqdomain, gpio);
				pr_info("MCP23016: irq_find_mapping: gpio = %ld, child_irq = %d\n", gpio, child_irq);
				handle_nested_irq(child_irq);
			}
		}
	}

	return (irqreturn_t)IRQ_HANDLED;
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

		mcp->ioport = value;

		return i2c_smbus_write_byte_data(mcp->client, reg_gpio, value);
	}

	return value;
}
/*----------------------------------------------------------------------------*/
static void mcp23016_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
	struct mcp23016 *mcp = to_mcp23016(gc);

	mutex_lock(&mcp->lock);
	mcp23016_set(mcp, offset, val);
	mutex_unlock(&mcp->lock);
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

	mcp->iodir = iodirval;

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
	int res;
	struct mcp23016 *mcp = to_mcp23016(gc);

	pr_info("MCP23016: Direction output\n");
	mutex_lock(&mcp->lock);
	res = mcp23016_direction(gc, offset, OUTPUT, val);
	mutex_unlock(&mcp->lock);
	return res;
}
/*----------------------------------------------------------------------------*/
static int mcp23016_direction_input(struct gpio_chip *gc,
                                    unsigned offset)
{
	int res;
	struct mcp23016 *mcp = to_mcp23016(gc);

	pr_info("MCP23016: Direction input\n");
	mutex_lock(&mcp->lock);
	res = mcp23016_direction(gc, offset, INPUT, 0);
	mutex_unlock(&mcp->lock);
	return res;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>
static void mcp23016_dbg_show(struct seq_file *s, struct gpio_chip *gc)
{
	struct mcp23016 *mcp = to_mcp23016(gc);
	int		t;
	unsigned	mask;

	for (t = 0, mask = 1; t < mcp->chip.ngpio; t++, mask <<= 1) {
		const char	*label;

		label = gpiochip_is_requested(&mcp->chip, t);
		if (!label)
			continue;

		seq_printf(s, " gpio-%-3d (%d) (%-12s) %s %s",
			mcp->chip.base + t, t, label,
			(mcp->iodir & mask) ? "in " : "out",
			(mcp->ioport & mask) ? "hi" : "lo");
		/* NOTE:  ignoring the irq-related registers */
		seq_puts(s, "\n");
	}
}
#else
#define mcp23s08_dbg_show	NULL
#endif
/*----------------------------------------------------------------------------*/
static int mcp23016_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	struct mcp23016 *mcp;
	int retval;
	//struct platform_device *pdev = to_platform_device(dev);
	//struct st_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	pr_info("MCP23016: Starting module\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("MCP23016: Error i2c <i2c_check_functionality> \n");
		return -EIO;
	}

	mcp = devm_kzalloc(&client->dev, sizeof(*mcp), GFP_KERNEL);
	if (!mcp) {
		pr_err("MCP23016: Error mem <devm_kzalloc> \n");
		return -ENOMEM;
	}

	mutex_init(&mcp->lock);

	mcp->chip.label = client->name;
	mcp->chip.base = -1;
	mcp->chip.parent = &client->dev;
	mcp->chip.owner = THIS_MODULE;
	mcp->chip.ngpio = GPIO_NUM;
	mcp->chip.can_sleep = true;
	mcp->chip.get = mcp23016_get_value;
	mcp->chip.set = mcp23016_set_value;
	mcp->chip.direction_output = mcp23016_direction_output;
	mcp->chip.direction_input = mcp23016_direction_input;
	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		mcp->chip.dbg_show = mcp23016_dbg_show;
		pr_info("MCP23016: Activate mcp23016_dbg_show \n");
	}

	mutex_init(&mcp->irq_lock);

	mcp->client = client;

	//client->irq = platform_get_irq(pdev, 0);

	/* Do we have an interrupt line? Enable the irqchip */
	if (client->irq) {
		strcat(irqName, dev_name(&client->dev));

		pr_info("MCP23016: fucn - devm_request_threaded_irq\n");
		retval = devm_request_threaded_irq(&client->dev, client->irq, NULL,
												mcp23016_irq,
												IRQF_ONESHOT |
												IRQF_TRIGGER_FALLING |
												IRQF_SHARED,
												irqName, mcp);
		if (retval) {
			pr_err("MCP23016: Error irq <devm_request_threaded_irq>, unable to request IRQ#%d: %d\n", client->irq, retval);
			goto fail;
		}

		pr_info("MCP23016: fucn - gpiochip_irqchip_add\n");
		// gpiochip_irqchip_add
		// gpiochip_irqchip_add_nested
		retval = gpiochip_irqchip_add_nested(&mcp->chip,
											&mcp23016_irq_chip,
											0, handle_level_irq, IRQ_TYPE_NONE);
		if (retval) {
			pr_err("cannot add irqchip\n");
			goto fail;
		}

		pr_info("MCP23016: fucn - gpiochip_set_chained_irqchip\n");
		//gpiochip_set_chained_irqchip
		//gpiochip_set_nested_irqchip
		gpiochip_set_nested_irqchip(&mcp->chip,
									&mcp23016_irq_chip,
									client->irq);

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

	retval = gpiochip_add(&mcp->chip);
	if(retval)
		goto fail;

	i2c_set_clientdata(client, mcp);
	return 0;

fail:
	pr_err("MCP23016: Can't setup chip\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
static int mcp23016_remove(struct i2c_client *client)
{
	struct mcp23016 *mcp = i2c_get_clientdata(client);

	mutex_lock(&mcp->lock);
	i2c_smbus_write_byte_data(client, IODIR0, 0x00);
	i2c_smbus_write_byte_data(client, IODIR1, 0x00);
	i2c_smbus_write_byte_data(client, GP0, 0x00);
	i2c_smbus_write_byte_data(client, GP1, 0x00);
	mutex_unlock(&mcp->lock);

	gpiochip_remove(&mcp->chip);
	synchronize_irq(client->irq);

	pr_info("MCP23016: Removing module\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static const struct of_device_id mcp23016_ids[] = {
	{ .compatible = "microchip,mcp23016", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mcp23016_ids);
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
MODULE_AUTHOR("Dmitry Domnin");
