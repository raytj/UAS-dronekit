sudo apt-get update -y
sudo apt-get upgrade -y
if command -v python3.7 &>/dev/null;
then
	sudo apt-get install build-essential tk-dev libncurses5-dev libncursesw5-dev libreadline6-dev libdb5.3-dev libgdbm-dev libsqlite3-dev libssl-dev libbz2-dev libexpat1-dev liblzma-dev zlib1g-dev libffi-dev -y
	cd /home/pi
	wget https://www.python.org/ftp/python/3.7.0/Python-3.7.0.tar.xz
	tar xf Python-3.7.0.tar.xz
	cd Python-3.7.0
	./configure
	make -j 4
	sudo make altinstall
	cd ..
	sudo rm -r Python-3.7.0
	rm Python-3.7.0.tar.xz
	sudo apt-get --purge remove build-essential tk-dev libncurses5-dev libncursesw5-dev libreadline6-dev libdb5.3-dev libgdbm-dev libsqlite3-dev libssl-dev libbz2-dev libexpat1-dev liblzma-dev zlib1g-dev libffi-dev -y
	sudo apt-get autoremove -y
	sudo apt-get clean
fi
sudo pip3 install opencv-python
sudo apt-get install python3-dev python-wxgtk3.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml libxml2-dev git -y
pip3 install MAVProxy
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
sudo adduser pi dialout
cd ../..
command apt-cache policy dronekit
git clone https://github.com/raytj/dronekit-python.git
cd dronekit-python
pip3 install -r install_requires.txt
python3.7 setup.py build
python3.7 setup.py install
pip3 install digi-xbee
