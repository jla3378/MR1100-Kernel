#include <linux/module.h>
#include "usb_netlink_base.h"

struct sock *nl_sk = NULL;
#define NETLINK_USB_SWITCH 24
#define NETLINKBASE_MAJOR 227
#define SET_VERSION_INFO 1
#define SET_SKU_INFO 2

static struct usb_cdrom_verfile_ioctl_type swupdate_ver_info;
static struct usb_sku_info_ioctl_type sku_info;

static int netlinkbase_open ( struct inode * , struct file * ); 
static int netlinkbase_release ( struct inode * , struct file * ); 
static long netlinkbase_ioctl( struct file *file, unsigned int command ,unsigned long arg);

void sendnlmsg(char *message, int msg_len, unsigned int usr_pid)  
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int rc;
	int len = NLMSG_SPACE(msg_len);
	u32 portid;

	printk(KERN_ERR "recvd msg of len =%d for processid = %u\n",  msg_len,usr_pid);
	if(!message)
		return;
	if(!nl_sk)
		return;
	if(usr_pid == 0)
		return;
	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb){
		printk(KERN_ERR "net_link: allocate failed.\n");
		return;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, msg_len, 0);
	
	portid = NETLINK_CB(skb).portid;   /* netlink portid */
	NETLINK_CB(skb).portid = 0;        /* from kernel */
	NETLINK_CB(skb).dst_group = 0;     /* unicast */     
	
	memcpy(NLMSG_DATA(nlh), message, msg_len);
	rc = netlink_unicast(nl_sk, skb, usr_pid, MSG_DONTWAIT);
	if (rc < 0)
		printk(KERN_ERR "net_link: can not unicast skb (%d)\n", rc);
	else
		printk("net_link: send is ok.\n");
	 return;
}
EXPORT_SYMBOL(sendnlmsg);

void getcdromverinfo(struct usb_cdrom_verfile_ioctl_type *info)
{
	if (!info)
		return;

	memcpy(info, &swupdate_ver_info, sizeof(swupdate_ver_info));
}
EXPORT_SYMBOL(getcdromverinfo);

void getskuinfo(struct usb_sku_info_ioctl_type *info)
{
	if (!info)
		return;

	memcpy(info, &sku_info, sizeof(sku_info));
}
EXPORT_SYMBOL(getskuinfo);

void nl_data_ready (struct sk_buff *__skb)
{
	return;
}

static ssize_t netlinkbase_write (struct file *file, const char __user *buf, size_t count,loff_t *offset)
{
	int ret = 0;
	return ret;
}

static int netlinkbase_open(struct inode *inode , struct file *file)
{
	if (!try_module_get(THIS_MODULE))
	{
		return -ENODEV;
	}
	return 0;  
} 

static long netlinkbase_ioctl( struct file *file, unsigned int command ,unsigned long arg)
{
	int ret = 0;

	switch(command)
	{
		case SET_VERSION_INFO:
			memset(&swupdate_ver_info, 0x00, sizeof(swupdate_ver_info));
			ret = copy_from_user(&swupdate_ver_info, (void __user*)arg, 
						sizeof(swupdate_ver_info));
			if( ret )
			{
				return -1;
			}
			printk(KERN_INFO"[%s:%d] SIPC1 = %u, SIVer1 = %u SIPC2 = %u SIver2 = %u", __FILE__, __LINE__,swupdate_ver_info.os1_sipc,swupdate_ver_info.os1_siver,swupdate_ver_info.os2_sipc, swupdate_ver_info.os2_siver);
			break;
		case SET_SKU_INFO:
			memset(&sku_info, 0x00, sizeof(sku_info));
			ret = copy_from_user(&sku_info, (void __user*)arg, 
						sizeof(sku_info));
			if( ret )
			{
				return -1;
			}
			break;
		default:
			break;
	}
	return 0;
}

static int netlinkbase_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);
	return 0;
}

static struct file_operations netlinkbase_ctl_fops = { 
	owner:	THIS_MODULE,
	open: 	netlinkbase_open , 
	read: 	NULL,
	write:	 netlinkbase_write,
	unlocked_ioctl: 	netlinkbase_ioctl ,
	release:	 netlinkbase_release , 
};

int netlink_init(void)
{
	int err=0;
	struct netlink_kernel_cfg cfg = {
		.input	= nl_data_ready,
	};
	
	nl_sk = netlink_kernel_create(&init_net,
						NETLINK_USB_SWITCH,
						&cfg);

	if (!nl_sk) 
	{
		printk(KERN_ERR "net_link: Cannot create netlink socket.\n");
		if (nl_sk != NULL)
		{
			sock_release(nl_sk->sk_socket);
		}
		return 0;
	}
	
	err = register_chrdev(NETLINKBASE_MAJOR, "basever",  &netlinkbase_ctl_fops);
	if(err < 0)
	{
		printk("fail to register the basever device\n");   	
		return -1;
	}
	return 0;
}

void free_netlink_rc(void)
{
	if (nl_sk != NULL)
	{
		sock_release(nl_sk->sk_socket);
	}
	unregister_chrdev(NETLINKBASE_MAJOR, "basever");
}

module_init(netlink_init);
module_exit(free_netlink_rc);

MODULE_DESCRIPTION("USB NETLINK base");
MODULE_AUTHOR("Chetan Karia");
MODULE_LICENSE("GPL");
