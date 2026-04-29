#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xf3ebda75, "__platform_driver_register" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x122c3a7e, "_printk" },
	{ 0xee67ed5, "devm_gpiod_get_index" },
	{ 0xa6b98038, "gpiod_to_irq" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xcf825a00, "platform_driver_unregister" },
	{ 0xb43f9365, "ktime_get" },
	{ 0xa8fa7cc2, "param_ops_int" },
	{ 0x47e64c59, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "D02A75A46939B1589033A20");
