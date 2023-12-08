#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/timekeeping.h>
#include <linux/device.h>

#define DEVICE_NAME "rear_encoder"
#define CLASS_NAME "spd_encoder_input"

/* GPIO descriptors for encoder */
static struct gpio_desc *enc_gpio;

/* enumeration of input sources, in this case interrupt controller to which gpio 21 is mapped to */
static int irq_num;

// Time variables
long curent_time = 0;
long last_time = 0;
long time_diff = 0;
// Current speed
long speed;

module_param(speed, long, S_IRUGO);

//Maybe we can give this the S_IRUGO attribute and just read it in python?
static int triggers = 0;

static irqreturn_t encoder_isr(int irq, void *dev_id) {
    int enc_state = gpiod_get_value(enc_gpio);
	
	 // if(triggers == 100) {
     curent_time = ktime_get_real_ns();
    time_diff = curent_time - last_time;
    last_time = curent_time;
    // }	
    if(time_diff > 70000){
	    speed = time_diff;
    }
    triggers++;
	
	// This should always be 0...
	// But it's detecting some 1s
    printk("speed %ld\n", speed);


    // handled
    return IRQ_HANDLED;
}

/**
 * @brief Called whenever user space code opens the character device
 * 	we don't need anything here
 */
static int encoder_open(struct inode *inodep, struct file *filp) {
	
	//printk(KERN_INFO "wow!\n");
	
    return 0;
}


/**
 * @brief init
 */
static int encoder_probe(struct platform_device *pdev) {
	int ret;
    enc_gpio = devm_gpiod_get(&pdev->dev, "userbutton", GPIOD_IN);
	
	// Set up the encoder ISR
    irq_num = gpiod_to_irq(enc_gpio);
    ret = request_irq(irq_num, encoder_isr, IRQF_TRIGGER_FALLING, "encoder_isr", pdev);
    // Notify and free resources if  unable to request interrupt
    if (ret) {
        printk("nightmare nightmare nightmare\n");
    	gpiod_put(enc_gpio);
        return ret;
    }
	
	//How to debounce?
	//Pi reads multiple triggers for every real triggers
	//we could potentially not debounce and just deal with some error, not sure, will have to test this
	
	
	//THIS FAILS WITH ERROR -524, i.e. debouncing not supported by processor????
	
    ret = gpiod_set_debounce(enc_gpio, 10000000);

	
	printk(KERN_INFO "wow cant believe it worked lol\n");
	
    return 0;
}

/**
 * @brief Free resources when module is removed
 */
static int encoder_remove(struct platform_device *pdev) {
	// Free the IRQ
    free_irq(irq_num, pdev);
	
    // Dispose of GPIO descriptors
    gpiod_put(enc_gpio);
	

	printk("byebye :( \n");
	
    return 0;
}

/* Match to device tree */
static const struct of_device_id match_to_devtree[] = {
    { .compatible = "encoder-gpio" },
    { },
};

/* Platform driver */
static struct platform_driver encoder_driver = {
    .probe = encoder_probe,
    .remove = encoder_remove,
    .driver = {
        .name = "kms",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(match_to_devtree),
    },
};



module_platform_driver(encoder_driver);

MODULE_DESCRIPTION("determine speed from encoder reading");
MODULE_AUTHOR("chris zhou");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:encoder_driver");
