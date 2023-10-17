# BART-Lab Radiomap Dataset
The BRAT-Lab radiomap dataset is high-fidelity simulated dataset via Altair WinProp: https://web.altair.com/winprop-telecom

The landscape map is accessible from OpenStreetMap: https://www.openstreetmap.org/

The dataset is available via: [BRAT-Lab Dataset](https://www.dropbox.com/scl/fo/kua86qnk1aukt8i4f2rma/h?rlkey=j82kiq4dgaw3l0bzbeyfvv4oi&dl=0)

The dataset consists of 2000 coarse resolution radiomaps in 5 frequency bands, and 100 fine resolution radiomaps in 5 frequency bands with size 5000 × 5000.

## Information of Coarse Resolution Radiomap:
The coarse resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in the United States of America, for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a 300 × 300 regular grid. The height of each building is set as 10 meters. 
Each transmitter contains three antennas (model 720842A2) with height as 35 meters and initial power as 46.00 dBm. 
5 frequency brands used for each antenna are 1750, 2750, 3750, 4750, 5750 MHz.

<img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1604_urbanmap.png?raw=true" width="200" height="200" align="center">
<img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1604_radiomap.png?raw=true" width="300" height="200" align="center">
<img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1700_urbanmap.png?raw=true" width="200" height="200" align="center">
<img src="https://github.com/BRATLab-UCD/Radiomap-Data/blob/main/examples/3750MHz_1700_radiomap.png?raw=true" width="300" height="200" align="center">

5 samples are in the folder /coarse-resolution. The code of radiomap generation is in /code.

## Information of Fine Resolution Radiomap:
The fine resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in the United States of America, for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a 5000 × 5000 regular grid. The height of each building is set as 10 meters. 
Each transmitter contains three antennas (model 720842A2) with height as 35 meters and initial power as 46.00 dBm. 
The frequency used for each antenna is 1750, 2750, 3750, 4750, 5750 MHz. 

[Example图片 Environment Map] [Example图片 Radiomap]

1 sample is in the folder /fine-resolution.

## Instruction of Generating your Radiomaps via Our Codes
Put some information about how to customize our codes for radiomap generation, i.e., parameters of antennas.

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
