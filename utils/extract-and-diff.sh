#!/bin/bash
cd bundle-src
aws s3 cp s3://deepracer-managed-resources-us-east-1/deepracer-simapp.tar.gz .
tar xvf deepracer-simapp.tar.gz bundle.tar
CUR_DATE="nov-11"
PREV_DATE="sep-30"
mkdir "bundle-$CUR_FOLDER" && cd "bundle-$CUR_FOLDER"
tar xvf ../bundle.tar opt/install
find . -name __pycache__ | xargs rm -rf
find . -name *.pyc | xargs rm -rf
cd ..
rm  deepracer-simapp.tar.gz bundle.tar
diff -U 10 -Nr $PREV_FOLDER $CUR_FOLDER/ > compare-$CUR_DATE.diff

cd ..
cd bundle
patch -p 3 < bundle-src/compare-$CUR_DATE.diff