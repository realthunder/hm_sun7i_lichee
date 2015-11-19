/*
 *
 * Copyright(c) 2014-2016 Allwinnertech Co., Ltd.
 *      http://www.allwinnertech.com
 *
 * Author: liushaohua <liushaohua@allwinnertech.com>
 *
 * switch driver for headset detect
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <mach/sys_config.h>

unsigned int switch_irq_gpio = GPIOH(15);
unsigned int switch_irq = 0;
static int aw_irq_handle;

struct headset_switch_data {
	struct switch_dev sdev;
	struct device 		*dev;
	int state;
	struct work_struct  detect_headset;
};

static int headset_flag = 0;

static struct workqueue_struct *headset_detect_queue;
static script_item_u item_eint;
static int detect_irq = 0;
static struct headset_switch_data *headsetdev;
int screen0_output_type = 0;

static ssize_t switch_headset_print_state(struct switch_dev *sdev, char *buf)
{
	struct headset_switch_data	*switch_data =
		container_of(sdev, struct headset_switch_data, sdev);

	return sprintf(buf, "%d\n", switch_data->state);
}

static unsigned int headset_detect_handler(void *dev_id)
{
	struct headset_switch_data * switch_data = (struct headset_switch_data *)dev_id;
	if(0 == queue_work(headset_detect_queue,&(switch_data->detect_headset))){
		printk("[headset detect ]add work struct ");
	}
	return 0;
}

static void headset_switch_work(struct work_struct *work)
{
	struct headset_switch_data	*switch_data =
		container_of(work, struct headset_switch_data, detect_headset);
	unsigned int status;
	int ret = 0;

	status = gpio_get_value(switch_irq_gpio);
	if (status == 1) {
		printk("[headset] Plug out\n");
		switch_data->state = 0;
	}
	else if(status == 0) {
		printk("[headset] Plug in \n");
		switch_data->state = 1;
	}
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static int headset_switch_probe(struct platform_device *pdev)
{

	script_item_value_type_e  type;
	int ret = 0;
	int status=0;

	headsetdev = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!headsetdev){
		printk("headset_switch_probe: not enough memory for this device\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, (void *)headsetdev);
	headsetdev->sdev.name = "h2w";
	headsetdev->state = 0;
	headsetdev->sdev.print_state = switch_headset_print_state;
	headsetdev->dev = &pdev->dev;

	INIT_WORK(&headsetdev->detect_headset, headset_switch_work);
	headset_detect_queue = create_singlethread_workqueue("detect_headset");
	
 	ret = gpio_request(switch_irq_gpio,"switch gpio");
        if (0 != ret) {
                printk("request headset detect f aield\n");
        }

	ret = switch_dev_register(&headsetdev->sdev);
	if (ret < 0)
		goto err_dev_register;
	
        ret = sw_gpio_irq_request(switch_irq_gpio,TRIG_EDGE_DOUBLE,(peint_handle)headset_detect_handler,headsetdev);
        if (!ret) {
                printk("request headset detect irq faield\n");
        }
        aw_irq_handle = ret;


#if 0

  type = script_get_item("disp_init", "screen0_output_type", &item_eint);
  if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
		printk("[DP] script_get_item return screen0_output_type err\n");
	}else{
		screen0_output_type = item_eint.val;
	}
  
	type = script_get_item("audio_para", "audio_int_ctrl", &item_eint);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		printk("[DP] script_get_item return type err\n");
	}
	detect_irq = gpio_to_irq(item_eint.gpio.gpio);
	if (IS_ERR_VALUE(detect_irq)) {
		printk("[DP] map gpio to virq failed, errno = %d\n",detect_irq);
		return -EINVAL;
	}


	ret = request_irq(detect_irq,headset_detect_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "detect_headset",headsetdev);
	if (ret < 0) {
		printk("headset detect irq requsest irq faield\n");
		return -EINVAL;
	}

	ret = gpio_request(item_eint.gpio.gpio,NULL);
#endif



	status = gpio_get_value(switch_irq_gpio);
	if (status == 1) {
		printk("[headset Init] ear out\n");
		headsetdev->state = 0;
	}
	else if(status == 0) {
		printk("[headset Init] ear in \n");
		headsetdev->state= 1;
	}
	switch_set_state(&headsetdev->sdev,headsetdev->state);

	return 0;

err_dev_register:
	kfree(&headsetdev->sdev);

	return ret;
}

static int __devexit headset_switch_remove(struct platform_device *pdev)
{

	struct headset_switch_data *headsetdev = platform_get_drvdata(pdev);

	//should free gpio
	if(headset_detect_queue)
		destroy_workqueue(headset_detect_queue);
	 
        sw_gpio_irq_free(aw_irq_handle);
	switch_dev_unregister(&headsetdev->sdev);
	kfree(headsetdev);
	return 0;
}

static struct platform_driver headset_switch_driver = {
	.probe		= headset_switch_probe,
	.remove		= __devexit_p(headset_switch_remove),
	.driver		= {
		.name	= "switch-display-port",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device headset_switch_device = {
    .name = "switch-display-port",
};

static int __init headset_switch_init(void)
{
	int ret;
	if((ret = platform_device_register(&headset_switch_device)) < 0)
		return ret;
	if((ret = platform_driver_register(&headset_switch_driver)) < 0)
		return ret;
	return 0;
}

static void __exit headset_switch_exit(void)
{
	platform_driver_unregister(&headset_switch_driver);
	platform_device_unregister(&headset_switch_device);
}

module_init(headset_switch_init);
module_exit(headset_switch_exit);

MODULE_AUTHOR("sam");
MODULE_DESCRIPTION("HEADSET Switch driver");
MODULE_LICENSE("GPL");
