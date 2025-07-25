//
// SPDX-License-Identifier: CC0-1.0
//
// This example code file is released to the public under Creative Commons CC0.
// See https://creativecommons.org/publicdomain/zero/1.0/legalcode
//
// To the extent possible under law, LEAP 71 has waived all copyright and
// related or neighboring rights to this PicoGK Example Code.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//


using PicoGK;


namespace Leap71
{
    using ShapeKernel;
    using Rover;

    namespace RoverExamples
    {
        class WheelShowCase
        {
            /// <summary>
            /// Generates one of the four preset rover wheel variants.
            /// Exports the final geometry as an STL file.
            /// </summary>
            public static void PresetWheelTask()
            {
                // Step 1: Choose a wheel variant 1-4
                // RoverWheel oWheel   = new Wheel_01();
                RoverWheel oWheel = new Wheel_02();
                // RoverWheel oWheel = new Wheel_03();
                // RoverWheel oWheel = new Wheel_04();


                // Step 2: Generate
                Voxels voxWheel     = oWheel.voxConstruct();


                // Step 3: Show and Export
                Uf.Wait(1f);
                Library.oViewer().RemoveAllObjects();
                Sh.PreviewVoxels(voxWheel, Cp.clrRock);
                Sh.ExportVoxelsToSTLFile(voxWheel, Sh.strGetExportPath(Sh.EExport.STL, "RoverWheel"));
            }

            /// <summary>
            /// Generates a randomized rover wheel variant.
            /// Exports the final geometry as an STL file.
            /// Exports a screenshot depicting the raw elements.
            /// Exports a screenshot depicting the final geometry.
            /// </summary>
            public static void RandomWheelTask()
            {
                uint nIndex         = 0;
                RandomWheel oWheel  = new RandomWheel(nIndex);
                Voxels voxWheel     = oWheel.voxConstruct();

                Uf.Wait(1f);
                Library.oViewer().RemoveAllObjects();
                Sh.PreviewVoxels(voxWheel, Cp.clrRandom());
                Library.oViewer().RequestScreenShot(Sh.strGetExportPath(Sh.EExport.TGA, $"RandomWheel_{nIndex}_Final"));
                Sh.ExportVoxelsToSTLFile(voxWheel, Sh.strGetExportPath(Sh.EExport.STL, "RandomRoverWheel"));
            }
        }
    }
}