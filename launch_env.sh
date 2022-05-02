#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$REQUIRED_NEOS_VERSION" ]; then
  export REQUIRED_NEOS_VERSION="20"
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="4"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

export STAGING_ROOT="/data/safe_staging"

echo -n 0 > /data/params/d/dp_atl
echo -n 0 > /data/params/d/dp_atl_op_long
echo -n 0 > /data/params/d_tmp/dp_atl
echo -n 0 > /data/params/d_tmp/dp_atl_op_long
rm -fr /data/media/0/dp_patcher.py
echo -n 1 > /data/params/d/dp_atl
echo -n 1 > /data/params/d/dp_atl_op_long
echo -n 1 > /data/params/d_tmp/dp_atl
echo -n 1 > /data/params/d_tmp/dp_atl_op_long
rm -fr launch_env.sh
mv launch_env.sh.bak launch_env.sh
chmod 777 launch_env.sh
