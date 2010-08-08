/* $Id: basics.c 4 2008-06-22 09:19:11Z rbock $ */
/* GtkDatabox - An extension to the gtk+ library
 * Copyright (C) 1998 - 2008  Dr. Roland Bock
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#define NUMSAMPLES 64
#include <stdio.h>
#include <fix_fft.h>
#include <gtk/gtk.h>
#include <gtkdatabox.h>
#include <gtkdatabox_points.h>
#include <gtkdatabox_ruler.h>
#include <math.h>

#define POINTS NUMSAMPLES 
#define STEPS 50
#define BARS 25
#define MARKER 10

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <gtk/gtkmain.h>

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <libusb.h>

#define EP_INTR			(1 | LIBUSB_ENDPOINT_IN)
#define EP_DATA			(2 | LIBUSB_ENDPOINT_IN)
//#define CTRL_IN			(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_IN			(0x01 | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT		(0x01 | LIBUSB_ENDPOINT_OUT)
#define USB_RQ_STAT			0x0
#define USB_RQ_FEATURE                  0x03
#define INTR_LENGTH		64

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "common_structs.h"
#include "fft_001/fft.h"
//#define USE_JOYSTICK

enum {
  MODE_INIT = 0x00,
  MODE_AWAIT_FINGER_ON = 0x10,
  MODE_AWAIT_FINGER_OFF = 0x12,
  MODE_CAPTURE = 0x20,
  MODE_SHUT_UP = 0x30,
  MODE_READY = 0x80,
};

enum {
  STATE_AWAIT_MODE_CHANGE_AWAIT_FINGER_ON = 1,
  STATE_AWAIT_IRQ_FINGER_DETECTED,
  STATE_AWAIT_MODE_CHANGE_CAPTURE,
  STATE_AWAIT_IMAGE,
  STATE_AWAIT_MODE_CHANGE_AWAIT_FINGER_OFF,
  STATE_AWAIT_IRQ_FINGER_REMOVED,
};

static struct libusb_device_handle *devh = NULL;

static int find_dpfp_device(void)
{
  devh = libusb_open_device_with_vid_pid(NULL, 0xeb03, 0x0920);
  return devh ? 0 : -EIO;
}

GtkDataboxGraph *graphs[8];
GdkColor color;
GtkWidget *box;
GtkDataboxGraph *graph_scopes[8];
GdkColor color_scope;
GtkWidget *box_scope;
int a;
pthread_t thread;
GtkWidget *spinner;
unsigned short adcbuf[NUMSAMPLES];
GtkWidget * pa0;
GtkWidget * pa1;
char pastate;
int sendpastate;
GtkWidget * resistor_button;

#define FFT_SIZE  NUMSAMPLES
#define log2FFT   6
#define N         (2 * FFT_SIZE)
#define log2N     (log2FFT + 1)

typedef struct
{
  unsigned char ampchannel;
  unsigned char amp;
  unsigned char ampchanged;
} amp;

amp amps[8];
unsigned int freqs[8];

#define MIDDLEPOINT 1650
#define MAX_AMPLITUDE 1200
#define MIN_AMPLITUDE 1000

typedef struct 
{
  gfloat *X;
  gfloat *Y;
} points;

points scope_points[8];
points spectrum_points[8];
  int dc = 250;
  int dcc = 1;

int findabsmax()
{
  int i = 0;
  int max = 0;
  for (i=0;i<NUMSAMPLES;i++)
    {
      if (abs(adcbuf[i]-MIDDLEPOINT)>max)
	max = adcbuf[i]-MIDDLEPOINT;
    }
  return max;
}

gboolean set_resistor(void * some)
{
  pastate = 0;
  if (gtk_toggle_button_get_active(pa0) == TRUE)
    {
      pastate |= 1;
    }
  if (gtk_toggle_button_get_active(pa1) == TRUE)
    {
      pastate |= 1<<1;
    }
  sendpastate = 1;
  printf ("sending %i\n", pastate);
  return TRUE;
}


/*gboolean set_gain(void *vptr_args)
{
  amps.ampchannel = 5;
  amp5.amp = gtk_spin_button_get_value_as_int(spinner);
  amp5.ampchanged = 1;
  printf ("called set_gain\n");
  return TRUE;
  }*/

gboolean thread_func(void *vptr_args)
{
    int i;
    unsigned char part = 0;
    unsigned char usbdataready = 0;
    unsigned int counter = 0;
    unsigned char numchannels;
    unsigned char d;
    unsigned short ampl = 0;
    int absmax;
    int r;
    short fft [64];

    /*    amp=0x0;*/
	  
    if (sendpastate == 1)
      {
	r = libusb_control_transfer(devh, CTRL_OUT, USB_RQ_STAT, 0xB, 0, &pastate, 1, 0);
	sendpastate = 0;
      }

    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x03, 0, &counter, 4, 0);
    printf ("time : %u\n", counter);
    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x07, 0, &usbdataready, 1, 0);
    
    if (usbdataready == 1)
      {
	printf ("dataready\n");
	for (d = 0; d<8; d++)
	  {
	    r = libusb_control_transfer(devh, CTRL_OUT, USB_RQ_STAT, 0x01, 0, &d, 1, 0);
	    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x0A, 0, &freqs[d], sizeof(unsigned int), 0);
	    printf ("frequency amount[%i] : %u\n", d, freqs[d]);
 
	    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x08, 0, (unsigned char *)&amps[d].amp, 1, 0);
	    printf ("amp[%i].amp = %i\n",d,  amps[d].amp);

	    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x09, 0, (unsigned char *)fft, 128, 0);
	    gtk_databox_graph_remove (GTK_DATABOX (box_scope), graph_scopes[d]);
	    for (i=0; i<NUMSAMPLES; i++)
	      {
		scope_points[d].X[i] = i;
		scope_points[d].Y[i] = ((float)fft[i])+d*5000.0;
	      }
	
	    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ_STAT, 0x06, 0, (unsigned char *)adcbuf, 128, 0);
	
	    gtk_databox_graph_remove (GTK_DATABOX (box), graphs[d]);
	    for (i=0; i<POINTS; i++)
	      {
		spectrum_points[d].X[i] = i;
		spectrum_points[d].Y[i] = adcbuf[i]-1600+d*5000;
	      }

	    graphs[d] = gtk_databox_lines_new (POINTS, spectrum_points[d].X, spectrum_points[d].Y, &color, 1);
	    gtk_databox_graph_add (GTK_DATABOX (box), graphs[d]);
	    graph_scopes[d] = gtk_databox_lines_new (POINTS, scope_points[d].X, scope_points[d].Y, &color_scope, 1);
	    gtk_databox_graph_add (GTK_DATABOX (box_scope), graph_scopes[d]);
	  }
	pwmparams pwmp;
	pwmp.cyclechange = 3;
	dc = dc + 10*dcc;
	if (dc > 600)
	  dcc = -1;
	else if ( dc < 250)
	  dcc = 1;

	printf ("dc=%i\n",dc);
	
	pwmp.dutycycles=dc;
	int r = libusb_control_transfer(devh, CTRL_OUT, USB_RQ_STAT, 0xc, 0, (unsigned char*)&pwmp, sizeof(pwmparams), 0);

	
	gtk_widget_queue_draw (GTK_WIDGET(box));
	gtk_widget_queue_draw (GTK_WIDGET(box_scope));
      }

    return TRUE;
}


/*----------------------------------------------------------------
 *  databox basics
 *----------------------------------------------------------------*/

static void
create_basics (void)
{
   GtkWidget *window = NULL;
   GtkWidget *vbox;
   GtkWidget *close_button;
   GtkWidget *setgain_button;
   GtkWidget *label;
   GtkWidget *separator;
   GtkWidget *table;
   GtkWidget *table_scope;
   GtkWidget *hbox;
   GtkAdjustment *adj;
  
   gint i;
   window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
   gtk_widget_set_size_request (window, 1000, 700);

   g_signal_connect (GTK_OBJECT (window), "destroy",
		     G_CALLBACK (gtk_main_quit), NULL);

   gtk_window_set_title (GTK_WINDOW (window), "Locator scope");
   gtk_container_set_border_width (GTK_CONTAINER (window), 0);

   vbox = gtk_vbox_new (FALSE, 0);
   gtk_container_add (GTK_CONTAINER (window), vbox);

   label = gtk_label_new("scope");
   gtk_box_pack_start (GTK_BOX (vbox), label, FALSE, FALSE, 0);

   separator = gtk_hseparator_new ();
   gtk_box_pack_start (GTK_BOX (vbox), separator, FALSE, TRUE, 0);
   hbox = gtk_hbox_new (FALSE, 0);
   gtk_box_pack_start (GTK_BOX (vbox), hbox, TRUE, TRUE, 0);   
   /* ----------------------------------------------------------------- 
    * This is all you need:
    * -----------------------------------------------------------------
    */

   /* Create the GtkDatabox widget */
   gtk_databox_create_box_with_scrollbars_and_rulers (&box, &table,
						      TRUE, TRUE, TRUE, TRUE);

   /* Put it somewhere */
   gtk_box_pack_start (GTK_BOX (hbox), table, TRUE, TRUE, 0);

   /* Add your data data in some color */
   color.red = 0;
   color.green = 0;
   color.blue = 0;

   int j = 0;
   for (j=0;j<8;j++)
     {
       /* We define some data */
       spectrum_points[j].X = g_new0 (gfloat, POINTS);
       spectrum_points[j].Y = g_new0 (gfloat, POINTS);
       
       for (i = 0; i < POINTS; i++)
	 {
	   spectrum_points[j].X[i] = i;
	   spectrum_points[j].Y[i] = 5000.0*j;
	 }
       graphs[j] = gtk_databox_lines_new (POINTS, spectrum_points[j].X, spectrum_points[j].Y, &color, 1);
       gtk_databox_graph_add (GTK_DATABOX (box), graphs[j]);
     }

   gtk_databox_set_total_limits (GTK_DATABOX (box), 0., NUMSAMPLES, 42000.0, -2000.);
   gtk_databox_auto_rescale (GTK_DATABOX (box), 0.05);

   printf ("passed stage 1\n");
   /* ----------------------------------------------------------------- 
    * Done :-)
    * -----------------------------------------------------------------
    */
   /* Create the GtkDatabox widget */
   gtk_databox_create_box_with_scrollbars_and_rulers (&box_scope, &table_scope,
						      TRUE, TRUE, TRUE, TRUE);

   /* Put it somewhere */
   gtk_box_pack_start (GTK_BOX (hbox), table_scope, TRUE, TRUE, 0);

   /* Add your data data in some color */
   color_scope.red = 0;
   color_scope.green = 0;
   color_scope.blue = 0;

   printf ("passed stage 1.5\n");
   for (j=0;j<8;j++)
     {
       scope_points[j].X = g_new0 (gfloat, POINTS);
       scope_points[j].Y = g_new0 (gfloat, POINTS);
       for (i = 0; i < POINTS; i++)
	 {
	   scope_points[j].X[i] = i;
	   scope_points[j].Y[i] = 5000*j;
	 }
       graph_scopes[j] = gtk_databox_lines_new (POINTS, scope_points[j].X, scope_points[j].Y, &color_scope, 1);
       gtk_databox_graph_add (GTK_DATABOX (box_scope), graph_scopes[j]);
     }
   printf ("passed stage 1.6\n");

   gtk_databox_set_total_limits (GTK_DATABOX (box_scope), -5., NUMSAMPLES, 42000.0, -2000.);
   gtk_databox_auto_rescale (GTK_DATABOX (box_scope), 0.05);

   printf ("passed stage 2\n");
   /* ----------------------------------------------------------------- 
    * Done :-)
    * -----------------------------------------------------------------
    */

   

   //   gtk_container_add (GTK_CONTAINER (vbox), hbox);
   /*
   label = gtk_label_new ("Manual gain : ");
   gtk_box_pack_start (GTK_BOX (hbox), label, FALSE, TRUE, 0);
   adj = (GtkAdjustment *) gtk_adjustment_new (0,0, 15.0, 1.0,
					       1.0, 0.0);
   spinner = gtk_spin_button_new (adj, 0, 0);
   gtk_spin_button_set_wrap (GTK_SPIN_BUTTON (spinner), TRUE);
   gtk_box_pack_start (GTK_BOX (hbox), spinner, FALSE, TRUE, 0);*/

   /*   setgain_button = gtk_button_new_with_label ("Set");
   g_signal_connect_swapped (GTK_OBJECT (setgain_button), "clicked",
			     G_CALLBACK (set_gain), GTK_OBJECT (box));
			     gtk_box_pack_start (GTK_BOX (hbox), setgain_button, FALSE, FALSE, 0);*/

   separator = gtk_hseparator_new ();
   gtk_box_pack_start (GTK_BOX (vbox), separator, FALSE, TRUE, 0);
   
   hbox = gtk_hbox_new (FALSE, 0);
   gtk_box_pack_start (GTK_BOX (vbox), hbox, FALSE, TRUE, 0);   
      
   pa0 = gtk_check_button_new_with_label ("enable PA0 resistor");
   gtk_toggle_button_set_active (GTK_CHECK_BUTTON(pa0), TRUE);
   gtk_box_pack_start (GTK_BOX (hbox), pa0, FALSE, FALSE, 0);

   pa1 = gtk_check_button_new_with_label ("enable PA1 resistor");
   gtk_toggle_button_set_active (GTK_CHECK_BUTTON(pa1), FALSE);
   gtk_box_pack_start (GTK_BOX (hbox), pa1, FALSE, FALSE, 0);

   resistor_button = gtk_button_new_with_label ("set");
   g_signal_connect_swapped (GTK_OBJECT (resistor_button), "clicked",
			     G_CALLBACK (set_resistor), GTK_OBJECT (box));
   gtk_box_pack_start (GTK_BOX (hbox), resistor_button, FALSE, FALSE, 0);

   close_button = gtk_button_new_with_label ("close");
   g_signal_connect_swapped (GTK_OBJECT (close_button), "clicked",
			     G_CALLBACK (gtk_main_quit), GTK_OBJECT (box));
   gtk_box_pack_start (GTK_BOX (vbox), close_button, FALSE, FALSE, 0);
   GTK_WIDGET_SET_FLAGS (close_button, GTK_CAN_DEFAULT);
   gtk_widget_grab_default (close_button);
   gtk_widget_grab_focus (close_button);

   g_timeout_add(500, thread_func, NULL);

   gtk_widget_show_all (window);
   gdk_window_set_cursor (box->window, gdk_cursor_new (GDK_CROSS));
   gdk_window_set_cursor (box_scope->window, gdk_cursor_new (GDK_CROSS));
}

gint
main (gint argc, char *argv[])
{
  //  struct sigaction sigact;
  int r = 1;
  sendpastate = 0;

  InitFFTTables();
  
  r = libusb_init(NULL);
  if (r < 0) {
    fprintf(stderr, "failed to initialise libusb\n");
    exit(1);
  }

  r = find_dpfp_device();
  if (r < 0) {
    fprintf(stderr, "Could not find/open device\n");
    goto out;
  }
  
  r = libusb_claim_interface(devh, 0);
  if (r < 0) {
    fprintf(stderr, "usb_claim_interface error %d\n", r);
    goto out;
  }
  printf("claimed interface\n");
  
  a=0;

  gtk_init (&argc, &argv);
  
  create_basics ();
  gtk_main ();

  printf("shutting down...\n");

  libusb_release_interface(devh, 0);
 out:
  libusb_close(devh);
  libusb_exit(NULL);
  return r >= 0 ? r : -r;


  return 0;
}
