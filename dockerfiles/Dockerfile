FROM ros:humble-ros-base-jammy
#FROM ros:humble

# Install SO dependencies
RUN apt-get update -qq && \
	apt-get install -y \
	build-essential \
	python3-pip -y \
    curl \
	--no-install-recommends terminator \
	&& rm -rf /var/lib/apt/lists/*

# Instal ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*


# Install python dependencies
RUN pip install setuptools==58.2.0

# Adding source command to (root)bashrc file for environment and in ws 
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
#Run echo "export ROS_DOMAIN_ID=101" >> /root/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Install BOOST 
RUN apt-get update \
  && apt-get install -y git \
                        g++ \
                        make \
                        wget \
	&& rm -rf /var/lib/apt/lists/*

# Download boost, untar, setup install with bootstrap and only do the Program Options library,
# and then install
RUN cd /home && wget http://downloads.sourceforge.net/project/boost/boost/1.74.0/boost_1_74_0.tar.gz \
  && tar xfz boost_1_74_0.tar.gz \
  && rm boost_1_74_0.tar.gz \
  && cd boost_1_74_0 \
  && ./bootstrap.sh --prefix=/usr/local \
  && ./b2 install \
  && cd /home \
  && rm -rf boost_1_74_0