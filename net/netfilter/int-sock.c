//##################### This module is written to disconnect all the sockets using a particular interface as soon as the interface goes down  ###########################


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/string.h>

#include <linux/socket.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/icmp.h>

#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>

#include <linux/notifier.h>
#include <net/tcp.h>
#include <net/inet_hashtables.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <net/udp.h>
#include <linux/inetdevice.h>



#define DEBUG(...) \
do { \
	if (intsock_debug_enabled) \
		printk(__VA_ARGS__); \
} while (0)

/** 
* @brief Turn on debug messages.
*/
static uint32_t intsock_debug_enabled=0;
module_param_named(debug_level, intsock_debug_enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "enable intsock debug.");

#define DEV_NAME "discsock"

extern struct inet_hashinfo tcp_hashinfo;

typedef struct worker
{
	struct work_struct work;
	spinlock_t poll_req_lock;
	struct list_head fin_head;
}worker_t;

static worker_t thr_ptr;

#define TCP_ESTABLISHED_TYPE	1
#define TCP_LISTENING_TYPE	2
#define UDP_TYPE		3	

typedef struct sock_entry
{
	struct list_head list;
	int type;
	int bucket;
	struct sock *sk;
}sock_entry_t;

// Push a socket to be disconnected in the global queue used by the daemon

void push_sock_for_reset(struct sock *sk,int type,int bucket)
{
	struct sock_entry *fin = NULL;

	fin = (struct sock_entry *)kmalloc(sizeof(struct sock_entry), GFP_ATOMIC);
	if (!fin) {
		printk(KERN_ERR "int-sock: unable to allocate memory\n");
		return;
	}

	fin->sk = sk;
	fin->type=type;
	fin->bucket=bucket;
	spin_lock_bh(&thr_ptr.poll_req_lock);
	list_add_tail(&fin->list, &thr_ptr.fin_head);
	spin_unlock_bh(&thr_ptr.poll_req_lock);
	return;
}


int check_tcp_established_socket(struct sock *this_sk, int bucket) {

	struct sock *sk;
	struct hlist_nulls_node *node;
	spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, bucket);

	spin_lock_bh(lock);
	sk_nulls_for_each(sk, node, &tcp_hashinfo.ehash[bucket].chain) {
		if (sk->sk_family != AF_INET) {
			continue;
		}
		if(sk==this_sk) {
			DEBUG("%s:Established socket %lx still in system list\n",
				__FUNCTION__,(unsigned long int)sk);
			spin_unlock_bh(lock);
			return 1;
		}
	}
	spin_unlock_bh(lock);
	return 0;
}


int check_tcp_listening_socket(struct sock *this_sk, int bucket) {

	struct sock *sk;
	struct hlist_nulls_node *node;
	struct inet_listen_hashbucket *ilb;

	ilb = &tcp_hashinfo.listening_hash[bucket];
	spin_lock_bh(&ilb->lock);
	sk = sk_nulls_head(&ilb->head);

	sk_nulls_for_each_from(sk, node) {
		if (sk->sk_family != AF_INET) {
			continue;
		}
		if(sk==this_sk) {
			DEBUG("%s:Listening socket %lx still in system list\n",__FUNCTION__,
				(unsigned long int)sk);
			spin_unlock_bh(&ilb->lock);
			return 1;
		}
	}
	spin_unlock_bh(&ilb->lock);
	return 0;
}


int check_udp_socket(struct sock *this_sk, int bucket) {
	struct sock *sk;
	struct hlist_nulls_node *node;
	struct udp_hslot *hslot;

	hslot = &udp_table.hash[bucket];
	spin_lock_bh(&hslot->lock);
	sk_nulls_for_each(sk, node, &hslot->head) {
		if (sk->sk_family != AF_INET)
			continue;

		if(sk==this_sk) {
			DEBUG("%s:UDP socket : %lx still in system list\n",__FUNCTION__,(unsigned long int)sk);
			spin_unlock_bh(&hslot->lock);
			return 1;
		}

	}

	spin_unlock_bh(&hslot->lock);
	return 0;
}


// Check if the sock address is from one of the active interfaces, if not kill it by returning 1

int check_addr_not_present(__be32 sock_addr) {
	struct net_device *dev = NULL;
	struct in_device *in_dev = NULL;
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	int ret = 1;

	if (!sock_addr || sock_addr == 0x0100007F)
		return 0;

	read_lock(&dev_base_lock);
	for_each_netdev	(&init_net, dev) {
		if(dev->flags & IFF_UP) {
			if ((in_dev = __in_dev_get_rtnl(dev)) != NULL) {
				for(ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
						ifap = &ifa->ifa_next) {
					if (!strcmp(dev->name, ifa->ifa_label))
					break; /* found */
				}
			}
			if(ifa) {
				DEBUG("%s:Interface %s, addr: %lx sock_addr: %lx\n",
						__FUNCTION__,dev->name,(unsigned long int)ifa->ifa_local,
						(unsigned long int)sock_addr);
				if(sock_addr == ifa->ifa_local) {
					ret = 0;
					break;
				}
			}
		}
	}

	read_unlock(&dev_base_lock);
	return ret;
}

int find_tcp_established_sockets(void) {
	int bucket;
	struct inet_sock *inet;
	__be32 src;


	for (bucket = 0; bucket <= tcp_hashinfo.ehash_mask; ++bucket) {
		struct sock *sk;
		struct hlist_nulls_node *node;
		spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, bucket);

		spin_lock_bh(lock);
		sk_nulls_for_each(sk, node, &tcp_hashinfo.ehash[bucket].chain) {
			if (sk->sk_family != AF_INET) {
				continue;
			}
			inet=inet_sk(sk);
			src = inet->inet_rcv_saddr;

			if((check_addr_not_present(src))  && (sk->sk_state == TCP_ESTABLISHED)) {
				DEBUG("%s:Established TCP socket found, addr: %lx\n",
					__FUNCTION__,(unsigned long int)sk);
				push_sock_for_reset(sk,TCP_ESTABLISHED_TYPE,bucket);
			}

		}
		spin_unlock_bh(lock);

	}

	return 0;
}

int find_tcp_listening_sockets(void) {
	int bucket;
	struct inet_sock *inet;
	__be32 src;

	for (bucket = 0; bucket < INET_LHTABLE_SIZE; ++bucket) {
		struct sock *sk;
		struct hlist_nulls_node *node;
		struct inet_listen_hashbucket *ilb;

		ilb = &tcp_hashinfo.listening_hash[bucket];
		spin_lock_bh(&ilb->lock);
		sk = sk_nulls_head(&ilb->head);

		sk_nulls_for_each_from(sk, node) {
			if (sk->sk_family != AF_INET) {
				continue;
			}
			inet=inet_sk(sk);
			src = inet->inet_rcv_saddr;

			if(check_addr_not_present(src)) {
				DEBUG("%s: Listening TCP socket found, addr: %lx\n",
					__FUNCTION__, (unsigned long int)sk);
				push_sock_for_reset(sk,TCP_LISTENING_TYPE,bucket);
			}
		}
		spin_unlock_bh(&ilb->lock);

	}

	return 0;
}

int find_udp_sockets(void) {
	struct sock *sk;
	unsigned int bucket;
	struct inet_sock *inet;
	__be32 src;

	for (bucket = 0; bucket <= udp_table.mask; bucket++) {
		struct hlist_nulls_node *node;
		struct udp_hslot *hslot = &udp_table.hash[bucket];

		if (hlist_nulls_empty(&hslot->head))
			continue;

		spin_lock_bh(&hslot->lock);
		sk_nulls_for_each(sk, node, &hslot->head) {
				DEBUG("%s:UDP socket: %lx, family %x\n",
					__FUNCTION__, (unsigned long int)sk, sk->sk_family);
				if (sk->sk_family != AF_INET)
					continue;
				inet=inet_sk(sk);
				src = inet->inet_rcv_saddr;

				if(check_addr_not_present(src)) {
					DEBUG("%s: UDP socket found, addr: %lx\n",
						__FUNCTION__,(unsigned long int)sk);
					push_sock_for_reset(sk,UDP_TYPE,bucket);
				}
		}
		spin_unlock_bh(&hslot->lock);
	}

	return 0;
}


// Worker callback

static void start_worker(struct work_struct *work)
{
	struct sock_entry *fin;
	struct sock *sk;


	spin_lock_bh(&thr_ptr.poll_req_lock);

	while(!list_empty(&thr_ptr.fin_head)) {
		fin = list_entry(thr_ptr.fin_head.next, struct sock_entry, list);
		list_del_init(&fin->list);
		spin_unlock_bh(&thr_ptr.poll_req_lock);
		sk=fin->sk;

		switch(fin->type) {
			case TCP_ESTABLISHED_TYPE:
				if(check_tcp_established_socket(sk,fin->bucket)) {
					lock_sock(sk);
					tcp_disconnect(sk,0);
					release_sock(sk);
				}
				break;
			case TCP_LISTENING_TYPE:
				if(check_tcp_listening_socket(sk,fin->bucket)) {
					lock_sock(sk);
					tcp_disconnect(sk,0);
					release_sock(sk);
				}
				break;

			case UDP_TYPE:
				if(check_udp_socket(sk,fin->bucket)) {
					lock_sock(sk);
					sk->sk_err=ENONET;
					sk->sk_error_report(sk);
					release_sock(sk);
				}
				break;
		}

		kfree(fin);
		spin_lock_bh(&thr_ptr.poll_req_lock);
	}
	spin_unlock_bh(&thr_ptr.poll_req_lock);
	return;		
}


// Initialize the structure used by daemon 

int init_worker(void)
{

	INIT_WORK(&thr_ptr.work,start_worker);
	INIT_LIST_HEAD(&thr_ptr.fin_head);
	spin_lock_init(&thr_ptr.poll_req_lock);
	return 0;
}

// Exit the daemon

int exit_worker(void)
{
	flush_scheduled_work();
	return 0;
}

void tear_down_interface_sockets(void) {
	find_tcp_established_sockets();
	find_tcp_listening_sockets();
	find_udp_sockets();
	schedule_work(&thr_ptr.work);
}

// Notifier event to capture the event of interface going down, and in that case
// push all sockets on the global queue

static int dev_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	switch (event) {
		case NETDEV_UNREGISTER:
			break;
		case NETDEV_REGISTER:
			break;
		case NETDEV_UP:
			break;
		case NETDEV_CHANGE:
		case NETDEV_DOWN:
			DEBUG("%s:Interface %s down/change , event : %s\n",__FUNCTION__,
				dev->name,(event==NETDEV_DOWN)?"NETDEV_DOWN":"NETDEV_CHANGE");
			if(!strcmp(dev->name,"lo"))
				break;
			tear_down_interface_sockets();
			break;
		default:
			break;
	}
	return NOTIFY_DONE;
}

struct notifier_block dev_notifier = {
	dev_notifier_event,
	NULL,
	0
};

/*
 * Verify TCP socket
 */
static int verify_tcp_socket(struct sock *sk_to_verify)
{
	int bucket;

	for (bucket = 0; bucket <= tcp_hashinfo.ehash_mask; ++bucket) {
		struct sock *sk;
		struct hlist_nulls_node *node;
		spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, bucket);

		spin_lock_bh(lock);
		sk_nulls_for_each(sk, node, &tcp_hashinfo.ehash[bucket].chain) {
			if (sk->sk_family != AF_INET) {
				continue;
			}
			if (sk == sk_to_verify) {
				spin_unlock_bh(lock);
				return 1;
			}
		}
		spin_unlock_bh(lock);
	}
	return 0;
}

/*
 * Verify UDP socket
 */
static int verify_udp_socket(struct sock *sk_to_verify)
{
	struct sock *sk;
	unsigned int bucket;

	for (bucket = 0; bucket <= udp_table.mask; ++bucket) {
		struct hlist_nulls_node *node;
		struct udp_hslot *hslot = &udp_table.hash[bucket];

		spin_lock_bh(&hslot->lock);
		sk_nulls_for_each(sk, node, &hslot->head) {
			if (sk->sk_family != AF_INET)
				continue;
			if (sk == sk_to_verify) {
				spin_unlock_bh(&hslot->lock);
				return 1;
			}
		}
		spin_unlock_bh(&hslot->lock);
	}
	return 0;
}


int kdev_ioctl(struct inode* inode, struct file* file, unsigned int cmd,
		unsigned long arg)
{
	struct sock *sk = (struct sock *) arg;

	if (!file || !inode || !arg) {
		return -1;
	}

	switch(cmd)
	{
		case 1:
			if (!verify_tcp_socket(sk)) {
				printk("KILL_TCP_SOCK: invalid sk %p\n", sk);
				return -1;
			}
			lock_sock(sk);
			tcp_disconnect(sk,0);
			release_sock(sk);
			break;
		case 2:
			if (!verify_udp_socket(sk)) {
				printk("KILL_UDP_SOCK: invalid sk %p\n", sk);
				return -1;
			}
			lock_sock(sk);
			sk->sk_err=ENONET;
			sk->sk_error_report(sk);
			release_sock(sk);
			break;
		default:
			break;
	}
	return 0;
}

struct file_operations kdev_fops =
{
	open: NULL,
	release: NULL,
	read: NULL,
	write: NULL,
	ioctl: kdev_ioctl
};

static int major_dev;
/* Initialisation routine */
static int __init interface_chk_init_module(void)
{

	memset(&thr_ptr,0,sizeof(worker_t));
	init_worker();

	register_netdevice_notifier(&dev_notifier);
	major_dev=register_chrdev(0,DEV_NAME,&kdev_fops);	// Registering as a char driver for ioctl call

	return 0;
}

/* Cleanup routine */
static void interface_chk_exit_module(void)
{
	exit_worker();
	unregister_netdevice_notifier(&dev_notifier);
	unregister_chrdev(major_dev,DEV_NAME);
}


module_init(interface_chk_init_module);
module_exit(interface_chk_exit_module);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Shuts down the sockets when the interface goes down or on an ioctl request");
MODULE_LICENSE("GPL");


