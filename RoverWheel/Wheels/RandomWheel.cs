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
		public class RandomWheel : RoverWheel
		{
            protected uint m_nIndex;

            /// <summary>
            /// Generates a randomized rover wheel for 3d-printing.
            /// The wheel is always designed with respect to the absolute origin.
            /// All key shape parameters, layers and elements are selected randomly.
            /// </summary>
            public RandomWheel(uint nIndex = 0)
			{
                m_nIndex = nIndex;


                //Step 0: Reset static parameters
                m_aFullUpperHeightPoints    = null;
                m_aFullLowerHeightPoints    = null;
                m_aUpperHeightFrames        = null;
                m_aLowerHeightFrames        = null;
                m_aInnerRadiusFrames        = null;
                m_aOuterRadiusFrames        = null;


                //Step 1: Define key dimensions
                m_fOuterRadius      = Uf.fGetRandomLinear(80, 200);
                m_fHubRadius        = Uf.fGetRandomLinear(30, 50);
                m_fRefWidth         = Uf.fGetRandomLinear(40, 90);


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
                    float fHubRatio                 = Uf.fGetRandomLinear(0.4f, 0.9f);
                    float fRoundRatio               = Uf.fGetRandomLinear(0.5f, 0.9f);
                    List<Vector3> aControlPoints    = new List<Vector3>();
                    aControlPoints.Add(new Vector3(fHubRatio * m_fRefWidth, 0f, 0.0f));
                    aControlPoints.Add(new Vector3(1.0f * m_fRefWidth,      0f, fRoundRatio));
                    aControlPoints.Add(new Vector3(1.0f * m_fRefWidth,      0f, 1.0f));
                    aControlPoints.Add(new Vector3(0.0f * m_fRefWidth,      0f, 1.0f));
                    ControlPointSpline oSpline = new ControlPointSpline(aControlPoints);
                    m_aFullUpperHeightPoints = oSpline.aGetPoints(100);
                    m_aFullUpperHeightPoints = m_aFullUpperHeightPoints.GetRange(0, 100);
                }
            }

            public override Voxels voxConstruct()
            {
                Voxels voxTread             = voxGetRandomTread();


                //set random layers
                List<WheelLayer> aLayers    = new List<WheelLayer>();
                bool bAddLayers             = true;
                float fStartRatio           = 0.05f;
                float fEndRatio             = 0.97f;
                Voxels voxSolidInnerLayer   = voxGetLayer(0f, fStartRatio);
                Voxels voxSolidOuterLayer   = voxGetLayer(fEndRatio, 1f);

                while (bAddLayers == true)
                {
                    WheelLayer sLayer = sGetRandomLayer(fStartRatio);
                    aLayers.Add(sLayer);

                    if (sLayer.m_fEndLengthRatio >= fEndRatio)
                    {
                        sLayer.m_fEndLengthRatio    = fEndRatio;
                        bAddLayers                  = false;
                    }
                    fStartRatio = sLayer.m_fEndLengthRatio;
                }


                //apply random elements per layer
                List<Voxels> aVoxelList = new List<Voxels>();
                foreach (WheelLayer sLayer in aLayers)
                {
                    WheelElements oElements = oGetRandomWheelElements(sLayer);
                    Voxels voxElements      = oElements.voxConstruct();
                    aVoxelList.Add(voxElements);
                }


                //assemble
                Voxels voxWheel             = Sh.voxUnion(voxSolidInnerLayer, voxTread);
                voxWheel                    = Sh.voxUnion(voxWheel, Sh.voxUnion(aVoxelList));
                voxWheel                    = Sh.voxUnion(voxWheel, voxSolidOuterLayer);


                Library.oViewer().RequestScreenShot(Sh.strGetExportPath(Sh.EExport.TGA, $"RandomWheel_{m_nIndex}_Raw"));
                Uf.Wait(0.1f);
                return voxWheel;
            }

            /// <summary>
            /// Returns a wheel tread as a voxelfield that randomly selects tread pattern and profile vs. solid layer.
            /// </summary>
            protected Voxels voxGetRandomTread()
            {
                //select tread pattern
                ITreadPattern xPattern      = xGetRandomTreadPattern();
                WheelTread		oTread		= new WheelTread(m_aOuterRadiusFrames, xPattern);

                //choose between solid tread or profile (inverse)
                bool bProfile               = Uf.bGetRandomBool();
                Voxels voxTread             = null;
                if (bProfile == true)
                {
                    voxTread = oTread.voxGetProfile();
                    return voxTread;
                }
                else
                {
                    float fOutwardsThickness    = Uf.fGetRandomLinear(1, 4);
                    float fInwardsThickness     = Uf.fGetRandomLinear(1, 2);
                    voxTread                    = oTread.voxGetTreadLayer(fOutwardsThickness, fInwardsThickness);
                    return voxTread;
                }
            }

            /// <summary>
            /// Returns a randomly selected tread pattern.
            /// </summary>
            protected ITreadPattern xGetRandomTreadPattern()
            {
                int iPatternIndex           = Math.Min(2, (int)(Uf.fGetRandomLinear(0, 3)));
                if (iPatternIndex == 0)
                {
                    ITreadPattern xPattern  = new TreadPattern_01();
                    return xPattern;
                }
                else if (iPatternIndex == 1)
                {
                    ITreadPattern xPattern  = new TreadPattern_02();
                    return xPattern;
                }
                else
                {
                    ITreadPattern xPattern  = new TreadPattern_03();
                    return xPattern;
                }
            }

            /// <summary>
            /// Returns a random wheel layer with the specified start radius ratio.
            /// </summary>
            protected WheelLayer sGetRandomLayer(float fStartRatio)
            {
                float fEndRatio         = MathF.Min(0.97f, fStartRatio + Uf.fGetRandomLinear(0.05f, 0.55f));
                WheelLayer sLayer       = new WheelLayer(this, fStartRatio, fEndRatio);
                return sLayer;
            }

            /// <summary>
            /// Returns a randomly selected type of wheel elements for the specified wheel layer.
            /// </summary>
            protected WheelElements oGetRandomWheelElements(WheelLayer sLayer)
            {
                uint nSymmetry          = (uint)(Uf.fGetRandomLinear(8, 30));
                float fWallThickness    = Uf.fGetRandomLinear(1f, 3f);

                int iElementIndex = Math.Min(4, (int)(Uf.fGetRandomLinear(0, 5)));
                if (iElementIndex == 0)
                {
                    WheelElements oElements = new EgyptianStruts(sLayer, nSymmetry, fWallThickness);
                    return oElements;
                }
                else if (iElementIndex == 1)
                {
                    WheelElements oElements = new RectHoles(sLayer, nSymmetry, fWallThickness);
                    return oElements;
                }
                else if (iElementIndex == 2)
                {
                    WheelElements oElements = new RosettaStruts(sLayer, nSymmetry, fWallThickness);
                    return oElements;
                }
                else if (iElementIndex == 3)
                {
                    WheelElements oElements = new SpiralStruts(sLayer, nSymmetry, fWallThickness);
                    return oElements;
                }
                else
                {
                    WheelElements oElements = new TubeStruts(sLayer, nSymmetry, fWallThickness);
                    return oElements;
                }
            }
        }
	}
}