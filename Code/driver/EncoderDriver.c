#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>

// Timer variables
ktime_t now;
static ktime_t lastTime;

// GPIO variables
unsigned int irq_enc_num;
struct gpio_desc *enc;
static int enc_count = 0;
module_param(enc_count, int, 0644);

/**
 * @brief Interrupt service routine is called, when interrupt is triggered
 */
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	s64 time_diff_ns;

	// Verify debounce time
	now = ktime_get();
	time_diff_ns = ktime_to_ns(ktime_sub(now, lastTime));

	// Debounce is 1ms because we have 20-hole wheel
	if(time_diff_ns < 10000000)
	{
		printk("Pressed is ignored\n");
		return IRQ_HANDLED;
	}

	// Count encoder
    enc_count++;
	lastTime = ktime_get();

  return IRQ_HANDLED;
}

// probe function
static int enc_probe(struct platform_device *pdev)
{
  	// Initiate Button
  	struct device *dev = &pdev->dev;
  	enc = devm_gpiod_get_index(dev, "enc", 0, GPIOD_IN);
  
  	// Register Button IRQ
  	irq_enc_num = gpiod_to_irq(enc);
  	if(request_irq(irq_enc_num, gpio_irq_handler, IRQF_TRIGGER_FALLING, "enc_irq", NULL) != 0){
		printk("Error!\nCan not request interrupt nr.: %d\n", irq_enc_num);
		return -1;
	}
	printk("Sucessfully insert Encoder module\n");
	return 0;
}

// remove function
static void enc_remove(struct platform_device *pdev)
{
	// Release IRQ
  	free_irq(irq_enc_num, NULL);
  	printk("Removing encoder\n");
}

static struct of_device_id matchy_match[] = {
    { .compatible = "speed_encoder"},
    {/* leave alone - keep this here (end node) */},
};

// platform driver object
static struct platform_driver enc_drv = {
	.probe	 = enc_probe,
	.remove	 = enc_remove,
	.driver	 = {
	       .name  = "Encoder driverr",
	       .owner = THIS_MODULE,
	       .of_match_table = matchy_match,
	},
};

module_platform_driver(enc_drv);

MODULE_DESCRIPTION("Encoder driver for Final Project - ELEC 553");
MODULE_AUTHOR("Owlt-onomous");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:encoder_driver");
