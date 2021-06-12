Since the custom gnss module is written for Gnu Radio 3.9 it is advised to install the latest version of the software.  
The best way to install latest gnu radio is to compile it from source, this ensures that all dependencies are installed.   
In order to do so, closely follow the guide, starting from Installing from Source: https://wiki.gnuradio.org/index.php/InstallingGR

After installation is complete follow this guide to add Python path where required: https://wiki.gnuradio.org/index.php/ModuleNotFoundError\
When asked to put exports into profile.d folder, any .sh file can be created in the folder with both export statements in it.  
This .sh file will be executed upon login. 

After the above steps are successfully completed, clone the repository: https://github.com/geraman21/gnss_grc\  
anywhere on your pc. Then open terminal in the repository folder an run the following commands to install the custom module in gnu radio:

```
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

After this is done click refresh in GRC GUI, and the custom module with all its block should be available via module selection list.