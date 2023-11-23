This is start of README for team 16595, 2024 Center Stage.
===== manual method , needs doing each time AS is started
to enable wifi direct in AS, need to navigate to  C:\Users\bruce\AppData\Local\Android\Sdk\platform-tools
then type  .\adb connect 192.168.43.1:5555
=====================================================

This sets up a tool to connect via wifi:
Here are the instructions on how to do it using a Control Hub over WiFi.

We will use the Android Debug Bridge (adb) app which automatically runs on the hub to provide 
communication between the hub and your PC over WiFi. 
Normally, you would open the terminal window (bottom of the AS screen) and issue commands 
to ADB to get this running. We will create two AS External Tools to make the process easier.

Android Studio allows for the creation of External Tools, 
which will appear in the main window’s Tools / ExternalTools menu.
These tools can do just about anything, and they can easily be used to run adb commands on the click of a button.

In Android Studio, open the main settings panel:

    On Mac OS, click “Android Studio”, then “Preferences…” (or simply press the hotkey “⌘,”).
    On Windows, click “File”, then “Settings…”.

In the settings panel:

    Expand the “Tools” item
    Click “External Tools” to open the External Tools list. The list is normally empty.
    For each of the following tools below, click the [+] button, and fill in the provided 
    fields (leaving all others unchanged). Once each tool screen is filled-in, click “OK” to save. 
    (Note: The “Program” field is the same, so the value can be cut and pasted to avoid re-typing.)
    Once all tools have been added, click the main settings panel’s “OK” button to save.

“Connect to ADB over WiFi”
Field 	Value
Name: 	Connect to ADB over WiFi
Program: 	$ModuleSdkPath$\platform-tools\adb.exe
Parameters: 	connect 192.168.43.1:5555

“Disconnect ADB”
Field 	Value
Name: 	Disconnect ADB
Program: 	$ModuleSdkPath$\platform-tools\adb.exe
Parameters: 	disconnect

With this done, connect your PC to the Control Hub over WiFi. When the PC has connected to the hub, 
run the Connect to ADB over WiFi external tool (Tools pull down menu->External Tools). 
When the command file has completed you are ready to use AS. 
The connection will remain until you disconnect from the hub's network. 
    Note: when running these external tool commands, you must have a java source file in your project 
    open with the cursor in that file when you execute the external tool.

Now, when you compile in AS, the networked Control Hub will be available as a deployment target. 
Compiled code will be sent to the hub over the network connection.

Note: while connected to the hub WiFi, your connection to the internet will not be available. 
You will need to switch networks when you need Internet access. 
If you do wish switch to another connection, run the Disconnect ADB tool before switching. 
When you switch back to the hub WiFi network, you will need to run the Connect to ADB over WiFi 
tool again to reestablish the connection between AS and the hub.

Note: If you do not disconnect adb before switching to away from the hub network, 
when you switch back to the hub network, and try to compile, in the window where you select 
the hub for deployment, it may be marked as [OFFLINE] and won't allow deployment. 
When this happens you will need to power the hub off and back on and reconnect.

Note: you can add the two external tools to the tool bar above the editing area. 
This left to you to figure out how, but to get started, right click on the tool bar.
=============================================================================

 