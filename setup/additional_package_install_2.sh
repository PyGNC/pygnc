echo "Running $0, this should only be run after additional_package_install_1.sh has successfully completed"
export CPPFLAGS="-I/usr/include/suitesparse"
mkdir ~/tmp_software
cd ~/tmp_software
# install qdldl from source as it fails when using pip
git clone https://github.com/osqp/qdldl-python.git
cd qdldl-python
git submodule init
git submodule update
python3 setup.py install --user
cd ~

pip3 install osqp

# install cvxopt from source to get glpk support
cd ~/tmp_software
git clone https://github.com/cvxopt/cvxopt.git
cd cvxopt
git checkout `git describe --abbrev=0 --tags`
export CVXOPT_BUILD_GLPK=1
python3 setup.py install --user
cd ~

pip3 install cvxpy

pip3 freeze > python_libraries.txt