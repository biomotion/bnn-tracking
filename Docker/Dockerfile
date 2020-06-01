ARG CUDA_V="10.0"
ARG CUDNN_V="7"

FROM pytorch/pytorch:nightly-devel-cuda${CUDA_V}-cudnn${CUDNN_V}


ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        git \
        curl \
        ca-certificates \
        build-essential \
        wget \
        python3-pip \
        python3-tk \
        python-opencv \
        git \
        libpcl-dev \
        python3-setuptools

WORKDIR /tmp

RUN python3 -m pip install -U pip && \
    pip3 install \
        jupyter \
        numpy \
        scipy \
        matplotlib \
        nuscenes-devkit \
        ninja yacs cython tqdm pyquaternion \
        cityscapesscripts \
        apex

RUN jupyter notebook --generate-config \
    && echo "c.NotebookApp.allow_root = True" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.password = 'sha1:b22dd1ea32ec:8d7e23c36485866b1f6d254be4eceb5fc389d0d1'" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.open_browser = False" >> /root/.jupyter/jupyter_notebook_config.py

RUN wget https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking/raw/master/requirements.txt -O 3dmot-requirements.txt && \
    wget https://github.com/nutonomy/nuscenes-devkit/raw/master/setup/requirements.txt -O nuscenese-requirements.txt && \
    pip3 install -r /tmp/3dmot-requirements.txt && \
    pip3 install -r /tmp/nuscenese-requirements.txt

# install maskrcnn
# WORKDIR /root
# ENV INSTALL_DIR=/root
# RUN git clone https://github.com/cocodataset/cocoapi.git && cd cocoapi/PythonAPI && \
#     python3 setup.py build_ext install
# RUN git clone https://github.com/mcordts/cityscapesScripts.git && cd cityscapesScripts/ && \
#     python3 setup.py build_ext install
# RUN git clone https://github.com/biomotion/apex.git && cd apex && \
#     python3 setup.py install --cuda_ext --cpp_ext
# RUN git clone https://github.com/facebookresearch/maskrcnn-benchmark.git && cd maskrcnn-benchmark && \
#     python3 setup.py build develop
# RUN git clone https://github.com/strawlab/python-pcl && cd python-pcl && \
#     python3 setup.py build_ext -i && \
#     python3 setup.py install

WORKDIR /

RUN git clone https://github.com/argoai/argoverse-api && cd argoverse-api && \
    wget https://s3.amazonaws.com/argoai-argoverse/hd_maps.tar.gz && \
    tar -zxvf hd_maps.tar.gz && \
    cd .. && pip3 install -e argoverse-api

    