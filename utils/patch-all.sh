VERS="cpu-avx cpu-avx2 cpu-gl-avx cpu-gl-avx2 gpu gpu-gl"
BASE="3.0.4"
TOBE=`cat VERSION`
for a in $VERS; do
	docker build . -t awsdeepracercommunity/deepracer-robomaker:$TOBE-$a -f docker/Dockerfile.localext --build-arg FROM_TAG=$BASE-$a
done
