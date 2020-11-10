#!/bin/bash
cd bundle-src
aws s3 cp s3://deepracer-managed-resources-us-east-1/deepracer-simapp.tar.gz .
tar xvf deepracer-simapp.tar.gz bundle.tar
CUR_DATE="nov-07"
PREV_DATE="nov-10"
mkdir "bundle-$CUR_DATE" && cd "bundle-$CUR_DATE"
tar xvf ../bundle.tar opt/install
find . -name __pycache__ | xargs rm -rf
find . -name *.pyc | xargs rm -rf
cd ..
rm  deepracer-simapp.tar.gz bundle.tar
diff -U 10 -Nr $PREV_FOLDER $CUR_DATE/ > compare-$CUR_DATE.diff

cd ..
cd bundle
patch -p 3 < ../bundle-src/compare-$CUR_DATE.diff

for a in $(cat ../bundle-src/compare-$CUR_DATE.diff | awk '/Binary files/ { print $5} ');
do
    FROM_FILE="../bundle-src/$a"
    TO_FILE_PART=$(echo $a |  cut -f4- -d/ )
    TO_FILE="bundle/$TO_FILE_PART"
    cp $FROM_FILE $TO_FILE
    echo $TO_FILE
done