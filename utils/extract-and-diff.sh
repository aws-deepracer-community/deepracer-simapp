#!/bin/bash
cd bundle-src
aws s3 cp s3://deepracer-melodic-managed-resources-us-east-1/deepracer-simapp.tar.gz .
tar xvf deepracer-simapp.tar.gz bundle.tar
CUR_DATE="2021-aug-29"
PREV_DATE="2021-jul-02"
mkdir "bundle-$CUR_DATE" && cd "bundle-$CUR_DATE"
tar xvf ../bundle.tar opt/install
find . -name __pycache__ | xargs rm -rf
find . -name *.pyc | xargs rm -rf
cd ..
rm  deepracer-simapp.tar.gz bundle.tar
diff -U 10 -Nr bundle-${PREV_DATE} bundle-${CUR_DATE}/ > compare-$CUR_DATE.diff

cd ..
cd bundle
patch -p 3 < ../bundle-src/compare-$CUR_DATE.diff
