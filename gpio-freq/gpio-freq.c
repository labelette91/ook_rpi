/***************************************************************************\

 Raspberry Pi GPIO frequency measurement.

 Copyright (c) 2014 Christophe Blaess

 This program is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License version 2 as published by
 the Free Software Foundation.

\***************************************************************************/
#include <linux/ktime.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
//#include <linux/version.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>

#include <linux/uaccess.h>

#include <linux/ioctl.h>

 
#define WR_VALUE _IOW('a','a',int32_t*)
#define RD_VALUE _IOR('a','b',int32_t*)

// ------------------ Default values ----------------------------------------

#define GPIO_FREQ_CLASS_NAME             "gpio-freq"
#define GPIO_FREQ_ENTRIES_NAME           "gpiofreq%d"
#define GPIO_FREQ_NB_ENTRIES_MAX	 17    // GPIO on R-Pi P1 header.

//------------------- Module parameters -------------------------------------

	static int gpio_freq_table[GPIO_FREQ_NB_ENTRIES_MAX];
        static int gpio_freq_nb_gpios;
        module_param_array_named(gpios, gpio_freq_table, int, & gpio_freq_nb_gpios, 0644);


// ------------------ Driver private data type ------------------------------
#define BUFFER_SZ			4096 
#define U32B unsigned long

struct gpio_freq_data ;

struct hrtimer_data {
    struct hrtimer  timer;
    struct gpio_freq_data* data;
    int   Int ;
};

struct gpio_freq_data {
		int gpio;
		int f_mode;
    ktime_t lastIrq_time;
    ktime_t lastTxTime;
    int  pRead;
    int  pWrite;
    int  wasOverflow;

	spinlock_t spinlock;
    int frequency;
    U32B lastDelta[BUFFER_SZ];
    
    U32B res[16];

   struct hrtimer_data timer_tx;
   int* txPulseDurationInMicros  ;
   int  txCount ;
   int  txNbData;

};


//test
void testData(struct gpio_freq_data * data)
{
    int i;
    return;
    for (i=0;i<16;i++){
        data->lastDelta[data->pWrite] = data->pWrite ;
        data->pWrite++ ;
        data->pWrite = data->pWrite % BUFFER_SZ ;
    }

}


// timer

int must_restart_timer = 0 ;


static enum hrtimer_restart handle_tx(struct hrtimer* timer)
{
struct hrtimer_data * ptimer = (struct hrtimer_data *) timer ;
struct gpio_freq_data* data = ptimer->data;
  enum hrtimer_restart result = HRTIMER_NORESTART;
  int dureeInMicros = 0 ;
  int gpio ;
  int pin  ;

    ktime_t current_time = ktime_get();
    ktime_t delta = ktime_sub_ns(current_time, data->lastTxTime);
    ktime_t deltaMicros  = ktime_to_us(delta);
    data->lastTxTime = current_time ;

  if (data->txPulseDurationInMicros !=0)
      dureeInMicros = data->txPulseDurationInMicros[data->txCount] ;
  gpio          = data->gpio;
  pin   = dureeInMicros  & 1 ;
  gpio_set_value( gpio, pin );

  // Restarts the TX timer.
//  if (must_restart_timer++<10)
  if ( (data->txCount<data->txNbData) && (dureeInMicros!=0))
  {
    ktime_t period = ktime_set(0, dureeInMicros*1000  );
    hrtimer_forward(timer, current_time, period);
    result = HRTIMER_RESTART;
  }
  else
  {
    //wake_up_process(data->sleeping_task);
  }
  
	printk(KERN_INFO "timer %d :%d micro %d : %lld \n",data->gpio , dureeInMicros , pin ,  deltaMicros );  
  data->txCount++;
  return result;
}

void initTxTimer(struct hrtimer_data * ptimer )
{
  // Initializes the  timer.
  hrtimer_init(&ptimer->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  ptimer->timer.function = &handle_tx;
}

void startTxTimer(struct hrtimer_data * ptimer , int periodInNs)
{
  
  ktime_t period = ktime_set(0, periodInNs  );
  
  // Starts the TX timer if it is not already running.
  if (!hrtimer_active(&ptimer->timer))
  {
    hrtimer_start(&ptimer->timer, period, HRTIMER_MODE_REL);
	printk(KERN_INFO "start timer %d\n" , ptimer->data->gpio);  
  }
}

static void cancelTxTimer(struct hrtimer_data * ptimer)
{
    hrtimer_cancel(&ptimer->timer);
} 

//---------

// ------------------ Driver private methods -------------------------------

static irqreturn_t gpio_freq_handler(int irq, void * filp);

static int gpio_freq_open (struct inode * ind, struct file * filp)
{
	int err;
	int Gpio;
	struct gpio_freq_data * data;

    //get gpio Pin
	Gpio = gpio_freq_table[iminor(ind)];

  printk(KERN_INFO "open inode %d:%d GPIO:%d mode:%d\n", imajor(ind) , iminor(ind),Gpio , filp->f_mode );
	
	data = kzalloc(sizeof(struct gpio_freq_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	
	data->gpio = Gpio ;
	data->f_mode = filp->f_mode ;
	spin_lock_init(& (data->spinlock));
		
//	err = gpio_request(Gpio, THIS_MODULE->name);
	err = gpio_request_one(Gpio, GPIOF_IN ,	 THIS_MODULE->name);
	if (err != 0) {
		printk(KERN_ERR "%s: unable to reserve GPIO %d\n", THIS_MODULE->name, Gpio);
		kfree(data);
		return err;
	}

//mode rd or r+ = Rd/Wr
	if ( filp->f_mode==29)
//	if ( ( filp->f_mode==29) || ( filp->f_mode==31) )
	{
		err = gpio_direction_input(Gpio);
		if (err != 0) {
			printk(KERN_ERR "%s: unable to set GPIO %d as input\n", THIS_MODULE->name, Gpio);
			gpio_free(Gpio);
			kfree(data);
			return err;
		}
		
		err = request_irq(gpio_to_irq(Gpio), gpio_freq_handler,
		                  IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
		                  THIS_MODULE->name, filp);
		if (err != 0) {
			printk(KERN_ERR "%s: unable to handle GPIO %d IRQ\n", THIS_MODULE->name, Gpio);
			gpio_free(Gpio);
			kfree(data);
			return err;
		}
	}
	else
//mode wr		
	if ( filp->f_mode==30)
	{
		err = gpio_direction_output(Gpio , 0 );
		if (err != 0) {
			printk(KERN_ERR "%s: unable to set GPIO %d as output\n", THIS_MODULE->name, Gpio);
			gpio_free(Gpio);
			kfree(data);
			return err;
		}
	}
	else
	{
			kfree(data);
			return -1 ;
		}

    data->timer_tx.data = data ;

    initTxTimer(&data->timer_tx);
		

	filp->private_data = data;
	testData(data);
	return 0;
}

static int gpio_freq_release (struct inode * ind,  struct file * filp)
{
	int err;
    //get gpio Pin
	int Gpio = gpio_freq_table[iminor(ind)];

	struct gpio_freq_data * data = filp->private_data;

  printk(KERN_INFO "close inode %d:%d GPIO:%d mode:%d\n", imajor(ind) , iminor(ind),Gpio , data->f_mode );

	err = gpio_direction_input(Gpio);
	if (err != 0) 
		printk(KERN_ERR "%s: unable to set GPIO %d as input\n", THIS_MODULE->name, Gpio);

	if ( data->f_mode==29)
//	if ( ( filp->f_mode==29) || ( filp->f_mode==31) )
{
	free_irq(gpio_to_irq(Gpio), filp);
}
	gpio_free(Gpio);

    cancelTxTimer(&data->timer_tx);

	kfree(filp->private_data);


	return 0;
}

int copy_data(struct gpio_freq_data * data , char * buffer, size_t length )
{
    int nb  ;
    int _error_count=0;
    if ( data->pRead != data->pWrite ) {

        /* copy data between read and write */
        if ( data->pRead < data->pWrite ) 
            nb = (data->pWrite-data->pRead)    ; 
        else
        /* copy data between read and end buffer */
            nb = (BUFFER_SZ-data->pRead)    ; 
        
        //compute size of long word
        length /=  sizeof(U32B);
        if (nb>= length )
            nb = length ;
        _error_count = copy_to_user(buffer,&data->lastDelta[data->pRead],nb * sizeof(U32B));
        data->pRead = (data->pWrite + nb) & (BUFFER_SZ-1);
	}
    return _error_count;
}

static int gpio_freq_read(struct file * filp, char * buffer, size_t length, loff_t * offset)
{

	struct gpio_freq_data * data = filp->private_data;
	//unsigned long irqmsk;

	// returns one of the line with the time between two IRQs
	// return 0 : end of reading
	// return >0 : size
	// return -EFAULT : error
	int _count=0;
	int _error_count=0;
  int nb ;
  int pwrite = data->pWrite;
  
//	spin_lock_irqsave(& (data->spinlock), irqmsk);
	if ( data->pRead != pwrite ) 
	{

        /* copy data between read and write */
        if ( data->pRead < pwrite ) 
            nb = (pwrite-data->pRead)    ; 
        else
        /* copy data between read and end buffer */
            nb = (BUFFER_SZ-data->pRead)    ; 
        
        //compute size of long word
        length /=  sizeof(U32B);
        if (nb>= length )
            nb = length ;
        _count =  nb * sizeof(U32B) ;   
        _error_count = copy_to_user(buffer,&data->lastDelta[data->pRead],_count );
        data->pRead = (data->pRead + nb) % (BUFFER_SZ);

  		//	testData(data);
   
	}
	
//	spin_unlock_irqrestore(& (data->spinlock), irqmsk);
/*
  if (_count>0)
  	printk(  "read %d %d",  _count , length );
*/
    if ( _error_count != 0 ) {
    	printk(KERN_ERR "RFRPI - copy_to_user");
        return -EFAULT;
    }

	return _count;

}

void transmit_code( int gpio , int * duree , size_t count )
{
	int i = 0 ;
	int pin = 0 ;
    unsigned long flags;
    //
    local_irq_save(flags); //Disabling all interrupts
	for ( i=0;i<count;i++){
	    pin = duree[i] & 1 ;
	    //printk(KERN_INFO  "%d: send %d %d\n" ,gpio, pin , duree[i]);
	    gpio_set_value( gpio, pin );
	    udelay(duree[i]);
	}
    local_irq_restore(flags);
}

static ssize_t gpio_freq_write(struct file *file, const char __user *buf,  size_t count, loff_t *pos)
{
    int * kbuf;
	int err;
    int i;
    unsigned int sendDuree =0 ;

	struct gpio_freq_data * data = file->private_data ;

	  if (count>= 4*1024)
			return -ENOMEM;
	
	 	//alloc data buffer
		kbuf = kzalloc(count, GFP_KERNEL);
		if (kbuf == NULL)
			return -ENOMEM;


    if (copy_from_user(kbuf, buf, count)) {   return -EFAULT;    }

    data->txPulseDurationInMicros   = kbuf;
    data->txNbData                  = count/4 ;
    data->txCount = 0 ;

//    disable_irq(gpio_to_irq(data->gpio)); //desable gpio pin interrupt

	err = gpio_direction_output(data->gpio , 0 );
	if (err != 0) {
		printk(KERN_ERR "%s: unable to set GPIO %d as output\n", THIS_MODULE->name, data->gpio);
		return err;
	}
/*
    startTxTimer(&data->timer_tx, 20000 );
    for ( i=0;i<data->txNbData;i++)    sendDuree+=data->txPulseDurationInMicros[i];
    while (data->txCount<=data->txNbData)
        mdelay(1);
*/
    transmit_code(	data->gpio , kbuf, count/4 );



    printk(GPIO_FREQ_ENTRIES_NAME ": send %d bytes %d us\n" ,data->gpio,count , sendDuree );
    kfree(kbuf);

	err = gpio_direction_input(data->gpio  );
	if (err != 0) {
		printk(KERN_ERR "%s: unable to set GPIO %d as input\n", THIS_MODULE->name, data->gpio);
		return err;
	}
//    enable_irq(gpio_to_irq(data->gpio));


    return count;
}

static irqreturn_t gpio_freq_handler(int irq, void * arg)
{
	struct gpio_freq_data * data;
	struct file * filp = (struct file *) arg;
   	ktime_t current_time;
    ktime_t delta;
   	unsigned long ns;
	int pinData;
   	current_time = ktime_get();
	
	if (filp == NULL)
		return -IRQ_NONE;

	data = filp->private_data;
	if (data == NULL)
		return IRQ_NONE;

	pinData = gpio_get_value(data->gpio);
	delta = ktime_sub_ns(current_time, data->lastIrq_time);
	ns = ktime_to_us(delta);

//  printk(KERN_INFO "pulse %ld\n", ns );

//    spin_lock(&(data->spinlock));

		//calcul etat du pulse que l'on mesure
		if (pinData == 1)
			//tranistion 0--1 : etat pulse = 0 : bit 0 = 0
			ns &=0xFFFE ;
		else
			//tranistion 1--0 : etat pulse = 1 : bit 0 = 1
			ns |= 1 ;
    
    data->lastDelta[data->pWrite] = ns;
   	//getnstimeofday(&data->lastIrq_time);
   	data->lastIrq_time = current_time;
   	
	data->pWrite = ( data->pWrite + 1 )  % (BUFFER_SZ);

	if (data->pWrite == data->pRead) {
		// overflow
		//data->pRead = ( data->pRead + 1 ) % (BUFFER_SZ);
		if ( data->wasOverflow == 0 ) {
	       printk(KERN_ERR "RFRPI - Buffer Overflow - IRQ will be missed");
	       data->wasOverflow = 1;
	    }
	} else {
		data->wasOverflow = 0;
	}
//    spin_unlock(&(data->spinlock));

	return IRQ_HANDLED;


}

static long gpio_freq_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	int32_t value = 0;
	int _error_count ;
	struct gpio_freq_data* data ;

	if (file == 0)
		return -1;
	 data = file->private_data;

	if (data->pRead != data->pWrite) {

		/* copy data between read and write */
		if (data->pRead < data->pWrite)
			value = (data->pWrite - data->pRead);
		else
			/* copy data between read and end buffer */
			value = (BUFFER_SZ - data->pRead);
	}

	switch (cmd) {
	case WR_VALUE:
		//		copy_from_user(&value, (int32_t*)arg, sizeof(value));
		printk(KERN_INFO "Value = %d\n", value);
		break;
	case RD_VALUE:
		_error_count = copy_to_user((int32_t*)arg, &value, sizeof(value));
		
    if ( _error_count != 0 ) {
    	printk(KERN_ERR "RFRPI - copy_to_user");
        return -EFAULT;
    }
		
		break;
	}
	return 0;
}


// ------------------ Driver private global data ----------------------------

static struct file_operations gpio_freq_fops = {
	.owner   =  THIS_MODULE,
	.open    =  gpio_freq_open,
	.release =  gpio_freq_release,
	.read    =  gpio_freq_read,
	.write   = gpio_freq_write,
	.unlocked_ioctl = gpio_freq_ioctl,
};




	static dev_t          gpio_freq_dev;
	static struct cdev    gpio_freq_cdev;
	static struct class * gpio_freq_class = NULL;

// ------------------ Driver init and exit methods --------------------------

static int __init gpio_freq_init (void)
{
	int err;
	int i;

	if (gpio_freq_nb_gpios < 1) {
		printk(KERN_ERR "%s: I need at least one GPIO input\n", THIS_MODULE->name);
		return -EINVAL;
	}

	err = alloc_chrdev_region(& gpio_freq_dev, 0, gpio_freq_nb_gpios, THIS_MODULE->name);
	if (err != 0)
		return err;
  printk(KERN_INFO "create region %d %d\n", MAJOR(gpio_freq_dev) , MINOR(gpio_freq_dev) );

	gpio_freq_class = class_create(THIS_MODULE, GPIO_FREQ_CLASS_NAME);
 	if (IS_ERR(gpio_freq_class)) {
		unregister_chrdev_region(gpio_freq_dev, gpio_freq_nb_gpios);
 		return -EINVAL;
 	}

	for (i = 0; i < gpio_freq_nb_gpios; i ++) {
		device_create(gpio_freq_class, NULL, MKDEV(MAJOR(gpio_freq_dev), i), NULL, GPIO_FREQ_ENTRIES_NAME, gpio_freq_table[i]);
		printk(KERN_INFO "create device %s%d \n",GPIO_FREQ_ENTRIES_NAME,gpio_freq_table[i] );
	}

	cdev_init(& gpio_freq_cdev, & gpio_freq_fops);

	err = cdev_add(& (gpio_freq_cdev), gpio_freq_dev, gpio_freq_nb_gpios);
	if (err != 0) {
		for (i = 0; i < gpio_freq_nb_gpios; i ++) 
			device_destroy(gpio_freq_class, MKDEV(MAJOR(gpio_freq_dev), i));
		class_destroy(gpio_freq_class);
		unregister_chrdev_region(gpio_freq_dev, gpio_freq_nb_gpios);
        printk(KERN_ERR "%s: error ading device\n", THIS_MODULE->name);
		return err;
	}

	return 0; 
}




void __exit gpio_freq_exit (void)
{
	int i;

	cdev_del (& gpio_freq_cdev);

	for (i = 0; i < gpio_freq_nb_gpios; i ++) {
		device_destroy(gpio_freq_class, MKDEV(MAJOR(gpio_freq_dev),  i));
		printk(KERN_INFO "delete device %s%d \n",GPIO_FREQ_ENTRIES_NAME,gpio_freq_table[i] );
	}
	class_destroy(gpio_freq_class);
	gpio_freq_class = NULL;

	unregister_chrdev_region(gpio_freq_dev, gpio_freq_nb_gpios);

}


module_init(gpio_freq_init);
module_exit(gpio_freq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("christophe.blaess@logilin.fr");

