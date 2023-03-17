from unittest import TestCase
from unittest.mock import patch, call
from ros_kvs_streamer.kvs_streamer import KvsStreamer
from ros_kvs_streamer.constants import KinesisStreamType
from sensor_msgs.msg import Image


@patch("rospy.Publisher")
@patch("rospy.Subscriber")
@patch("ros_kvs_streamer.kvs_streamer.Thread")
@patch("ros_kvs_streamer.kvs_streamer.Popen")
@patch("ros_kvs_streamer.kvs_streamer.ROSUtil")
class KvsStreamerTest(TestCase):
    def setUp(self) -> None:
        self._topic = "test_topic"
        self._namespace = "test_namespace"
        self._stream_name = "test_stream_name"
        self._stream_region = "test_stream_region"

    def test_start_kvs_node_dead(self, ros_util_mock, popen_mock,
                                 thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = False
        KvsStreamer.start_kvs(self._topic,
                              self._namespace,
                              self._stream_name,
                              self._stream_region)
        popen_mock.assert_has_calls([
            call("roslaunch ros_kvs_streamer kinesis_video_streamer.launch "
                 "topic:={} stream_name:={} stream_region:={} "
                 "__ns:={}".format(self._topic,
                                   self._stream_name,
                                   self._stream_region,
                                   self._namespace),
                 shell=True,
                 executable="/bin/bash"),
            call("roslaunch ros_kvs_streamer h264_video_encoder.launch "
                 "topic:={} "
                 "__ns:={}".format(self._topic,
                                   self._namespace),
                 shell=True,
                 executable="/bin/bash")])
        ros_util_mock.wait_for_rosnode.assert_called_once_with(
            alive_nodes=["/{}/kinesis_video_streamer".format(self._namespace),
                         "/{}/h264_video_encoder".format(self._namespace)])

    def test_start_kvs_node_live(self, ros_util_mock, popen_mock,
                                 thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = True
        KvsStreamer.start_kvs(self._topic,
                              self._namespace,
                              self._stream_name,
                              self._stream_region)
        popen_mock.assert_not_called()
        ros_util_mock.wait_for_rosnode.assert_not_called()

    def test_stop_kvs_node_live(self, ros_util_mock, popen_mock,
                                thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = True
        KvsStreamer.stop_kvs(self._namespace)
        popen_mock.assert_has_calls([
            call("rosnode kill /{}/kinesis_video_streamer".format(self._namespace),
                 shell=True,
                 executable="/bin/bash"),
            call("rosnode kill /{}/h264_video_encoder".format(self._namespace),
                 shell=True,
                 executable="/bin/bash")])
        ros_util_mock.wait_for_rosnode.assert_called_once_with(
            dead_nodes=["/{}/kinesis_video_streamer".format(self._namespace),
                        "/{}/h264_video_encoder".format(self._namespace)])

    def test_stop_kvs_node_dead(self, ros_util_mock, popen_mock,
                                thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = False
        KvsStreamer.stop_kvs(self._namespace)
        popen_mock.assert_not_called()
        ros_util_mock.wait_for_rosnode.assert_not_called()

    def test_start_kvs_node_dead_webrtc(self, ros_util_mock, popen_mock,
                                        thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = False
        KvsStreamer.start_webrtc(self._topic,
                                 self._namespace,
                                 self._stream_name,
                                 self._stream_region)
        popen_mock.assert_has_calls([
            call("roslaunch ros_kvs_streamer kinesis_webrtc_streamer.launch "
                 "topic:={} stream_name:={} stream_region:={} "
                 "__ns:={}".format(self._topic,
                                   self._stream_name,
                                   self._stream_region,
                                   self._namespace),
                 shell=True,
                 executable="/bin/bash"),
            call("roslaunch ros_kvs_streamer h264_video_encoder.launch "
                 "topic:={} "
                 "__ns:={}".format(self._topic,
                                   self._namespace),
                 shell=True,
                 executable="/bin/bash")])
        ros_util_mock.wait_for_rosnode.assert_called_once_with(
            alive_nodes=["/{}/kinesis_webrtc_streamer".format(self._namespace),
                         "/{}/h264_video_encoder".format(self._namespace)])

    def test_start_kvs_node_live_webrtc(self, ros_util_mock, popen_mock,
                                        thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = True
        KvsStreamer.start_webrtc(self._topic,
                                 self._namespace,
                                 self._stream_name,
                                 self._stream_region)
        popen_mock.assert_not_called()
        ros_util_mock.wait_for_rosnode.assert_not_called()

    def test_stop_kvs_node_live_webrtc(self, ros_util_mock, popen_mock,
                                       thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = True
        KvsStreamer.stop_webrtc(self._namespace)
        popen_mock.assert_has_calls([
            call("rosnode kill /{}/kinesis_webrtc_streamer".format(self._namespace),
                 shell=True,
                 executable="/bin/bash"),
            call("rosnode kill /{}/h264_video_encoder".format(self._namespace),
                 shell=True,
                 executable="/bin/bash")])
        ros_util_mock.wait_for_rosnode.assert_called_once_with(
            dead_nodes=["/{}/kinesis_webrtc_streamer".format(self._namespace),
                        "/{}/h264_video_encoder".format(self._namespace)])

    def test_stop_kvs_node_dead_webrtc(self, ros_util_mock, popen_mock,
                                       thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        ros_util_mock.is_ros_node_alive.return_value = False
        KvsStreamer.stop_webrtc(self._namespace)
        popen_mock.assert_not_called()
        ros_util_mock.wait_for_rosnode.assert_not_called()

    @patch.object(KvsStreamer, "start_kvs")
    def test_start(self, start_kvs_mock, ros_util_mock, popen_mock,
                   thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)
        kvs_stream.start()
        thread_mock.assert_not_called()
        ros_sub_mock.assert_not_called()
        ros_pub_mock.assert_not_called()
        start_kvs_mock.assert_called_once_with(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)

    @patch.object(KvsStreamer, "start_webrtc")
    def test_start_webrtc(self, start_webrtc_mock, ros_util_mock, popen_mock,
                          thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region,
            stream_type=KinesisStreamType.WEBRTC)
        kvs_stream.start()
        thread_mock.assert_not_called()
        ros_sub_mock.assert_not_called()
        ros_pub_mock.assert_not_called()
        start_webrtc_mock.assert_called_once_with(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)

    @patch.object(KvsStreamer, "start_kvs")
    @patch.object(KvsStreamer, "stop_kvs")
    def test_stop(self, stop_kvs_mock, start_kvs_mock, ros_util_mock, popen_mock,
                  thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)
        kvs_stream.start()
        kvs_stream.stop()
        stop_kvs_mock.assert_called_once_with(
            namespace=self._namespace)
        thread_mock.return_value.join.assert_not_called()

    @patch.object(KvsStreamer, "start_webrtc")
    @patch.object(KvsStreamer, "stop_webrtc")
    def test_stop_webrtc(self, stop_webrtc_mock, start_webrtc_mock, ros_util_mock, popen_mock,
                         thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region,
            stream_type=KinesisStreamType.WEBRTC)
        kvs_stream.start()
        kvs_stream.stop()
        stop_webrtc_mock.assert_called_once_with(
            namespace=self._namespace)
        thread_mock.return_value.join.assert_not_called()

    @patch.object(KvsStreamer, "start_kvs")
    def test_start_use_proxy(self, start_kvs_mock, ros_util_mock, popen_mock,
                             thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region,
            use_proxy_topic=True,
            publish_rate=1.0 / 15.0)
        kvs_stream.start()
        thread_mock.assert_called_once_with(target=kvs_stream._kvs_frame_publisher)
        thread_mock.return_value.start.assert_called_once()
        ros_sub_mock.assert_called_once_with(kvs_stream._topic,
                                             Image,
                                             kvs_stream._on_frame_received)
        ros_pub_mock.assert_called_once_with(kvs_stream._proxy_topic,
                                             Image,
                                             queue_size=1)
        start_kvs_mock.assert_called_once_with(
            topic=kvs_stream._proxy_topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)

    @patch.object(KvsStreamer, "start_webrtc")
    def test_start_use_proxy_webrtc(self, start_webrtc_mock, ros_util_mock, popen_mock,
                                    thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region,
            use_proxy_topic=True,
            publish_rate=1.0 / 15.0,
            stream_type=KinesisStreamType.WEBRTC)
        kvs_stream.start()
        thread_mock.assert_called_once_with(target=kvs_stream._kvs_frame_publisher)
        thread_mock.return_value.start.assert_called_once()
        ros_sub_mock.assert_called_once_with(kvs_stream._topic,
                                             Image,
                                             kvs_stream._on_frame_received)
        ros_pub_mock.assert_called_once_with(kvs_stream._proxy_topic,
                                             Image,
                                             queue_size=1)
        start_webrtc_mock.assert_called_once_with(
            topic=kvs_stream._proxy_topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region)

    @patch.object(KvsStreamer, "start_webrtc")
    @patch.object(KvsStreamer, "stop_webrtc")
    def test_stop_use_proxy(self, stop_webrtc_mock, start_webrtc_mock, ros_util_mock, popen_mock,
                            thread_mock, ros_sub_mock, ros_pub_mock) -> None:
        kvs_stream = KvsStreamer(
            topic=self._topic,
            namespace=self._namespace,
            stream_name=self._stream_name,
            stream_region=self._stream_region,
            use_proxy_topic=True,
            publish_rate=1.0 / 15.0,
            stream_type=KinesisStreamType.WEBRTC)
        kvs_stream.start()
        kvs_stream.stop()
        stop_webrtc_mock.assert_called_once_with(
            namespace=self._namespace)
        assert kvs_stream._thread_terminate_event.is_set()
        thread_mock.return_value.join.assert_called_once()
