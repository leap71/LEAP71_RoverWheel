//
// SPDX-License-Identifier: Apache-2.0
//
// The LEAP 71 ShapeKernel is an open source geometry engine
// specifically for use in Computational Engineering Models (CEM).
//
// For more information, please visit https://leap71.com/shapekernel
// 
// This project is developed and maintained by LEAP 71 - © 2023 by LEAP 71
// https://leap71.com
//
// Computational Engineering will profoundly change our physical world in the
// years ahead. Thank you for being part of the journey.
//
// We have developed this library to be used widely, for both commercial and
// non-commercial projects alike. Therefore, have released it under a permissive
// open-source license.
// 
// The LEAP 71 ShapeKernel is based on the PicoGK compact computational geometry 
// framework. See https://picogk.org for more information.
//
// LEAP 71 licenses this file to you under the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, THE SOFTWARE IS
// PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.   
//


namespace Leap71
{
	namespace Rover
	{
        /// <summary>
        /// Simple struct to store information about a layer within a wheel.
        /// </summary>
		public struct WheelLayer
        {
            public float         m_fStartLengthRatio;
            public float         m_fEndLengthRatio;
            public RoverWheel    m_oWheel;

            public WheelLayer(  RoverWheel oWheel,
                                float fStartLengthRatio,
                                float fEndLengthRatio)
            {
                m_oWheel            = oWheel;
                m_fStartLengthRatio = fStartLengthRatio;
                m_fEndLengthRatio   = fEndLengthRatio;
            }
        }
	}
}