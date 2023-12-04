# BART-Lab Radiomap Dataset
The BRAT-Lab radiomap dataset is high-fidelity simulated dataset via Altair WinProp: https://web.altair.com/winprop-telecom

The landscape map is accessible from OpenStreetMap: https://www.openstreetmap.org/

```diff <b> The dataset is available via </b>: [BRAT-Lab Dataset](https://www.dropbox.com/scl/fo/kua86qnk1aukt8i4f2rma/h?rlkey=j82kiq4dgaw3l0bzbeyfvv4oi&dl=0)

The dataset consists of 2000 coarse resolution radiomaps in 5 frequency bands, and 100 fine resolution radiomaps in 5 frequency bands with size 3000 × 5000.

## Information of Coarse Resolution Radiomap:
The coarse resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in the United States of America, for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a regular grid. The height of each building is set as 10 meters. 
Each transmitter contains three antennas (model 720842A2) with height as 30 meters and initial power as 46.00 dBm. 
5 frequency brands used for each antenna are 1750, 2750, 3750, 4750, 5750 MHz.

<table>
<tr>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1604_urbanmap.png?raw=true" width="400" height="300" align="center"></td>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1604_radiomap.png?raw=true" width="500" height="400" align="center"></td>
</tr>
<tr>
<td align="center">Urban Map1</td>
<td align="center">Radio Map1</td>
</tr>
</table>

<table>
<tr>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1700_urbanmap.png?raw=true" width="400" height="300" align="center"></td>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1700_radiomap.png?raw=true" width="500" height="400" align="center"></td>
</tr>
<tr>
<td align="center">Urban Map2</td>
<td align="center">Radio Map2</td>
</tr>
</table>

The code of radiomap generation is in /simulation.

## Information of Fine Resolution Radiomap:
The fine resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in the United States of America, for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a 3000 × 5000 regular grid. The height of each building is set as 10 meters. 
Each transmitter contains three antennas (model 720842A2) with height as 30 meters and initial power as 46.00 dBm. 
The frequency used for each antenna is 1750, 2750, 3750, 4750, 5750 MHz. 

<table>
<tr>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/5750MHz_2_urbanmap.png?raw=true" width="400" height="300" align="center"></td>
<td><img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/5750MHz_2_radiomap.png?raw=true" width="500" height="300" align="center"></td>
</tr>
<tr>
<td align="center">Urban Map</td>
<td align="center">Radio Map</td>
</tr>
</table>

## Instruction of Generating your Radiomaps via Our Codes
The code for generating data is stored in "Simulation". It is written in C++, and you will need to use CMake to set up the tasks. This code operates by invoking the API provided by Altair Feko for simulation purposes. To harness the full capabilities of the API used in the code, you will need to obtain a Winprop license through official channels. After that, You can adjust the code's parameters to suit your requirements. For more detailed information, refer to the official user manual at: https://2022.help.altair.com/2022.1.1/winprop/html/topics/winprop/user_guide/appendix/api/api_general_winprop_c.htm.

## Folders in the Repository
osm_data: This folder holds the information of 2000 sets of regions obtained from OpenStreetMap through the Overpass API.

odb_data: Data transformed from OSM files into a format suitable for processing with FEKO Winprop software.

coordinates: Latitude and longitude coordinates for the 2000 regions. For each line, data saved as (x1 y1 x2 y2 x3 y3).

buildings_positions: The relative positions of buildings within each region.

stations_positions: The relative positions of radar installations within each region.

antenna_patterns: The information about the chosen antenna.

airInterfaces: The parameters of 5G propagation model air interface.

radiomap_txt: The received power for the whole network in text file. For each line, data saved as (x y power).

radiomap_mat: The received power for the whole network in matrix, converted from 'radiomap_txt'.

## Acknowledgement
Please acknowledge the following paper if the dataset is useful for your research.

@article{zhang2023radiomap,<br/>
  title={Radiomap Inpainting for Restricted Areas based on Propagation Priority and Depth Map}, <br/>
  author={Zhang, Songyang and Yu, Tianhang and Choi, Brian and Ouyang, Feng and Ding, Zhi},<br/>
  journal={arXiv preprint arXiv:2305.15526}, <br/>
  year={2023} <br/>
}

## Contact
Please contact Tianhang Yu for more information regarding the BRAT-Lab Dataset if you have any questions.

Email: thgyu@ucdavis.edu
