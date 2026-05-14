# Tomu USB CDC-NCM (Ethernet NIC) + HTTP web server

This example implements a USB Ethernet adapter, using CDC-NCM, which is natively supported in Windows and Linux.
The program also implements a web server connected to this network, for the host OS to interact with.

## Supported features

 * ARP reply: sends the "server" MAC address matching its requested IP address.
 * ICMP Echo (Ping) reply.
 * A UDP packet with Rx/Tx stats is regularly sent to the host (unless disabled).
 * An HTTP (over TCP/IP port 80) web server serves an HTML page + favicon icon.
 * Unknown GET paths return a 404 page.
 * When a valid packet is received, the red LED is toggled (unless disabled).
 * When a packet is transmitted, the green LED is toggled (unless disabled).

## Usage

* Flash and connect the Tomu to the USB host.
* The host OS CDC-NCM driver will create a new network interface.
* Configure the new interface's IP to the default Host IP.
* Open a browser (or use 'wget', 'ping', etc.) to the default Server IP.

## Default IP addresses and ports

* Server IP: ``192.168.7.1``
* Host IP: ``192.168.7.2``
* HTTP Server TCP port: ``80``
* UDP stats packet port: ``1234``

## Main HTTP page

<html><body style='font-family:monospace;background:#111;color:#0f0'>
<pre style='font-family:monospace;background:#111;color:#0f0'>
<h1>Tomu NCM</h1><p>rx=3067 tx=1899<form action='/' method='GET'><p><input type='hidden' name='led' value='0'><label><input type='checkbox' name='led' value='1' checked> Rx/Tx LED</label></p><p><input type='hidden' name='udp' value='0'><label><input type='checkbox' name='udp' value='1' checked> UDP Stats</label></p><p><button type='submit' style='font-family:monospace; background:#222; color:#0f0; border:1px solid #0f0; padding:4px 14px; cursor:pointer;'>Apply</button></p></form></p><p>Uptime: 0:21:37</p><p><small><a href='https://github.com/im-tomu/tomu-quickstart/tree/master/usb-cdcncm' style='color:#0f0;'>USB CDC-NCM on EFM32HG309</a></small></p></body></html>
</pre>

## 404 HTTP page


<html><body style='font-family:monospace;background:#111;color:#0f0'><pre style='font-family:monospace;background:#111;color:#0f0'>                 ###      404       
  404          ##:-*#               
               #######              
                 ##           ##***#
            ############      #*--+#
        ##*+=------====++*##  ######
        ##.:--------------##    ### 
#####   ##.--+**+==+**=---##    ### 
#=-=*#  ##:=+..... ....==-##    ### 
######  ##-== :+*. .*+.==-##  ####  
  ##    ##-=+:.......:-+=-#######   
  ####  ##---=++++++++=---####      
    ######---------------=##        
       ###-------404-----=##    404 
        ##---------------=##        
        ####################        
  404       ####    #####           
           #*==*#  #*==*##          </pre></body></html>

