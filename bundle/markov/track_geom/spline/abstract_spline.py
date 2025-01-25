#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

'''This class defines an abstract spline'''
import abc

from scipy.interpolate import spalde

from markov.track_geom.constants import SPLINE_DEGREE
from markov.track_geom.track_data import TrackData

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})

class AbstractSpline(ABC):
    def __init__(self):
        self._track_data = TrackData.get_instance()
        self.build_spline()

    @property
    def lane(self):
        '''Lane getter
        '''
        return self._lane

    def build_spline(self):
        '''Build spline for track
        '''
        track_line, dists, spline = self._build_spline()
        self._lane = {"track_line": track_line,
                      "dists": dists,
                      "spline": spline}

    @abc.abstractmethod
    def _build_spline(self):
        '''Build spline for track

        Returns:
            tuple: input track lane, track lane point distance,
                  prepared track lane spline.

        Raises:
            NotImplementedError: Build spline method is not implemented
        '''
        raise NotImplementedError('Build spline method is not implemented')

    def eval_spline(self, dist):
        '''Use spline to generate point

        Args:
            dist (float): lane change dist

        Returns:
            spalde: Evaluate all derivatives of a B-spline.
            https://docs.scipy.org/doc/scipy-0.18.1/reference/generated/scipy.interpolate.spalde.html
        '''
        center_line = self._track_data.center_line
        min_dist = self._lane['spline'][0][SPLINE_DEGREE]
        max_dist = self._lane['spline'][0][-SPLINE_DEGREE-1]
        if dist < min_dist: dist += center_line.length
        if dist > max_dist: dist -= center_line.length
        return spalde(max(min_dist, min(dist, max_dist)), self._lane['spline'])
