#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/extcon.h>
#include <linux/mutex.h>


#define  GPIO_DOCK_ID  445  // SW31


struct dock_driver_data {
	struct device *dev;
	unsigned int gpio_id;
	int irq;
	struct work_struct detect_work;
	struct extcon_dev extcon;
	struct mutex state_lock;
	int id_state;
};


static void dock_detect_work(struct work_struct *work)
{
	int id_state = -1;
	struct dock_driver_data *dock = container_of(work, struct dock_driver_data, detect_work);

	mutex_lock(&dock->state_lock);

	id_state = gpio_get_value(dock->gpio_id);

	dev_dbg(dock->dev, "dock->id_state = %d, id_state = %d\n", dock->id_state, id_state);

	if (dock->id_state != id_state) {
		dev_info(dock->dev, "dock ID state changed: %d -> %d\n", dock->id_state, id_state);
		extcon_set_state(&dock->extcon, (id_state ? 0 : 1));
		dock->id_state = id_state;
	}

	mutex_unlock(&dock->state_lock);

    return;
}


static irqreturn_t dock_irq_handler(int irq, void *dev_id)
{
	struct dock_driver_data *dock = (struct dock_driver_data *)dev_id;
	schedule_work(&dock->detect_work);
    return IRQ_HANDLED;
}


static int dock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dock_driver_data *dock;
	int ret;

	dock = devm_kzalloc(dev, sizeof(*dock), GFP_KERNEL);
	if (dock == NULL) {
		dev_err(dev, "dock allocate memory failed\n");
		ret = -ENOMEM;
		goto PROBE_MEM_FAIL;
	}

	platform_set_drvdata(pdev, dock);
	dock->dev = dev;

	dock->gpio_id = GPIO_DOCK_ID;

	ret = gpio_request(dock->gpio_id, "dock_ID");
	if (ret < 0) {
		dev_err(dev, "[%s] request dock->gpio_id failed. ret = %d\n", __func__, ret);
		goto PROBE_GPIO_ID_FAIL;
	}

	// xmsxm @ 2019-10-24. Try to fix UI rotation issue caused by V3.09 BIOS. -start-
	ret = gpio_direction_input(GPIO_DOCK_ID);
	if (ret < 0) {
		dev_err(dev, "[%s] set dock->gpio_id direction input failed. ret = %d\n", __func__, ret);
		goto PROBE_GPIO_DIRECTION_FAIL;
	}
	// xmsxm @ 2019-10-24. Try to fix UI rotation issue caused by V3.09 BIOS. -end-

	INIT_WORK(&dock->detect_work, dock_detect_work);

	mutex_init(&dock->state_lock);

	dock->irq = gpio_to_irq(dock->gpio_id);
	dev_dbg(dev, "dock->irq = %d\n", dock->irq);

	ret = request_irq(dock->irq, dock_irq_handler, IRQ_TYPE_EDGE_BOTH, "dock_ID", dock);
	if (ret < 0) {
		dev_err(dev, "[%s] request dock->irq failed. ret = %d\n", __func__, ret);
		goto PROBE_IRQ_FAIL;
	}

	dock->extcon.name = "dock";
	ret = extcon_dev_register(&dock->extcon);
	if (ret < 0) {
		dev_err(dev, "[%s] extcon device register failed. ret = %d\n", __func__, ret);
		goto PROBE_EXTCON_FAIL;
	}

	dock->id_state = -1;
	schedule_work(&dock->detect_work);

	dev_info(dev, "dock probe successfully.");

	return 0;

PROBE_EXTCON_FAIL:
	free_irq(dock->irq, dock);
PROBE_IRQ_FAIL:
	mutex_destroy(&dock->state_lock);
	gpio_free(dock->gpio_id);
PROBE_GPIO_DIRECTION_FAIL:
PROBE_GPIO_ID_FAIL:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(dev, dock);
PROBE_MEM_FAIL:
	return ret;
}


static int dock_remove(struct platform_device *pdev)
{
	struct dock_driver_data *dock = platform_get_drvdata(pdev);
	extcon_dev_unregister(&dock->extcon);
	free_irq(dock->irq, dock);
	mutex_destroy(&dock->state_lock);
	gpio_free(dock->gpio_id);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, dock);
	return 0;
}


static int dock_resume(struct platform_device *pdev)
{
	struct dock_driver_data *dock = platform_get_drvdata(pdev);
	schedule_work(&dock->detect_work);
	return 0;
}


static struct acpi_device_id dock_acpi_match_table[] = {
	{ "DOCK0001" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, dock_acpi_match_table);


static struct platform_driver dock_driver = {
	.probe = dock_probe,
	.remove = dock_remove,
	.resume = dock_resume,
	.driver = {
		.name = "dock_onkyo_pds2734",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(dock_acpi_match_table),
	},
};


static int __init dock_init(void)
{
	return platform_driver_register(&dock_driver);
}

static void __exit dock_exit(void)
{
	platform_driver_unregister(&dock_driver);
}


module_init(dock_init);
module_exit(dock_exit);

MODULE_LICENSE("GPL");

