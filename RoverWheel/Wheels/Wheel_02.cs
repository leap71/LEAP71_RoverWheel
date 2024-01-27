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


using System.Numerics;
using PicoGK;


namespace Leap71
{
    using ShapeKernel;

    namespace Rover
    {
        /// <summary>
        /// Generates a the second variant of the preset rover wheels.
        /// </summary>
		public class Wheel_02 : RoverWheel
		{
            public Wheel_02()
			{
                //Step 0: Reset static parameters
                m_aFullUpperHeightPoints    = null;
                m_aFullLowerHeightPoints    = null;
                m_aUpperHeightFrames        = null;
                m_aLowerHeightFrames        = null;
                m_aInnerRadiusFrames        = null;
                m_aOuterRadiusFrames        = null;


                //Step 1: Define key dimensions
                m_fOuterRadius      = 120;
                m_fHubRadius        = 30;
                m_fRefWidth         = 60f;


                //Step 2: Construct contours
                SetFullUpperHeightPoint();
				SetFullLowerHeightPoints();
                m_oWheel		    = new BaseLens(	new LocalFrame(),
												    m_fRefWidth,
                                                    m_fHubRadius,
                                                    m_fOuterRadius);
                m_oWheel.SetHeight(oGetWidthModulation(m_aFullLowerHeightPoints), oGetWidthModulation(m_aFullUpperHeightPoints));
				SetInnerRadiusPoints();
				SetOuterPoints();
                ShowPlaneGrid();
				Library.oViewer().RemoveAllObjects();
            }

            protected override void SetFullUpperHeightPoint()
			{
				if (m_aFullUpperHeightPoints == null)
				{
					List<Vector3> aControlPoints = new List<Vector3>();
					aControlPoints.Add(new Vector3(0.6f * m_fRefWidth, 0f, 0.0f));
					aControlPoints.Add(new Vector3(1.0f * m_fRefWidth, 0f, 0.6f));
					aControlPoints.Add(new Vector3(1.0f * m_fRefWidth, 0f, 1.0f));
					aControlPoints.Add(new Vector3(0.0f * m_fRefWidth, 0f, 1.0f));
					ControlPointSpline oSpline	= new ControlPointSpline(aControlPoints);
                    m_aFullUpperHeightPoints		= oSpline.aGetPoints(100);
					m_aFullUpperHeightPoints		= m_aFullUpperHeightPoints.GetRange(0, 100);
                }
            }

            public override Voxels voxConstruct()
            {
                ITreadPattern xPattern	    = new TreadPattern_03();
                WheelTread oTread		    = new WheelTread(m_aOuterRadiusFrames, xPattern);
				Voxels voxTread				= oTread.voxGetProfile();

                WheelLayer sLayer_01	    = new WheelLayer(this, 0.02f, 0.20f);
                WheelLayer sLayer_02	    = new WheelLayer(this, 0.20f, 0.60f);
                WheelLayer sLayer_03	    = new WheelLayer(this, 0.60f, 0.80f);
                WheelLayer sLayer_04	    = new WheelLayer(this, 0.80f, 0.90f);
                WheelLayer sLayer_05	    = new WheelLayer(this, 0.90f, 1.00f);

                WheelElements oElements_01	= new RectHoles(sLayer_01, 20, 4);
                WheelElements oElements_02	= new RosettaStruts(sLayer_02, 10, 4);
                WheelElements oElements_03	= new RectHoles(sLayer_03, 10, 2);
                WheelElements oElements_04	= new RosettaStruts(sLayer_04, 50, 2);
                WheelElements oElements_05	= new RectHoles(sLayer_05, 20, 2);

                Voxels voxStruts_01		    = oElements_01.voxConstruct();
                Voxels voxStruts_02		    = oElements_02.voxConstruct();
                Voxels voxStruts_03		    = oElements_03.voxConstruct();
                Voxels voxStruts_04		    = oElements_04.voxConstruct();
                Voxels voxStruts_05		    = oElements_05.voxConstruct();

				Voxels voxWheel			    = voxGetLayer(0f, 0.02f);
				voxWheel				    = Sh.voxUnion(voxWheel, voxStruts_01);
                voxWheel				    = Sh.voxUnion(voxWheel, voxStruts_02);
                voxWheel				    = Sh.voxUnion(voxWheel, voxStruts_03);
                voxWheel				    = Sh.voxUnion(voxWheel, voxStruts_04);
                voxWheel				    = Sh.voxUnion(voxWheel, voxStruts_05);
				voxWheel				    = Sh.voxUnion(voxWheel, voxTread);
                return voxWheel;
            }
		}
	}
}