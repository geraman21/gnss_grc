This gnuradio module  requires Gnu Radio version 3.9 and above.  
For Linux based systems it is fairly simple to compile Gnu Radio from source, this ensures that all dependencies are installed.   
In order to do so, closely follow the guide, starting from Installing from Source: https://wiki.gnuradio.org/index.php/InstallingGR

Gnu Radio is also available on many software distributions. Check that the version is 3.9 and above before installing.

After installation is complete follow this guide to add Python path where required if necessary: https://wiki.gnuradio.org/index.php/ModuleNotFoundError
When asked to put exports into profile.d folder, any .sh file can be created in the folder with both export statements in it.  
This .sh file will be executed upon login. 

After the above steps are successfully completed, clone the repository: https://github.com/geraman21/gnss_grc  
anywhere on your pc. Then open terminal in the repository folder an run the following commands to install the custom module in gnu radio:

```
mkdir build
cd build
cmake ..
make
sudo make install
```
When gnu radio is installed not in the default folder, you have to specify the installation path. For example when using homebrew on mac use the following command:

```
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local/Cellar/gnuradio/3.9.2.0_1
```
Make sure to specify the correct installation folder according to the gnuradio version installed.

After installation is complete, open GRC GUI and click refresh button, the custom module with all its block should be available inside the module selection list.

receiver_test.grc is an example frow graph that calculates GPS L1 position using up to 6 observables. The source GPS signal can be found at: https://sourceforge.net/projects/gnss-sdr/files/data/2013_04_04_GNSS_SIGNAL_at_CTTC_SPAIN.tar.gz/download

Refer to GNSS-GRC_Software-defnined-gps-receiver_thesis.pdf for a detail description of the project.
