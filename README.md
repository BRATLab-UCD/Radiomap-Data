# BART-Lab Radiomap Dataset
The BRAT-Lab radiomap dataset is high-fidelity simulated dataset via Altair WinProp: https://web.altair.com/winprop-telecom

The landscape map is accessible from OpenStreetMap: https://www.openstreetmap.org/

The dataset is available via: [BRAT-Lab Dataset](https://www.dropbox.com/scl/fo/bfxkjk2vxtj3sou422zll/h?rlkey=1wahdlqi0kh1b21qdbtng4jf0&dl=0)

The dataset consists of [x] coarse resolution radiomaps in [X] frequency bands, and [X] fine resolution radiomaps with size [x × Y].

## Information of Coarse Resolution Radiomap:
The coarse resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in [XXXXXX], for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a [x × Y] regular grid. The height of each building is set as [X] meters. 
Each transmitter contains three antennas (model [720842A2]) with height as [35] meters and initial power as [46.00] dBm. 
The frequency used for each antenna is [2625] MHz.

[Example图片 Environment Map] [Example图片 Radiomap]
[Example图片 Environment Map] [Example图片 Radiomap]

5 samples are in the folder /coarse-resolution. The code of radiomap generation is in /code.

## Information of Fine Resolution Radiomap:
The fine resolution dataset is generated by the simulation tool Altair Feko. The dataset is generated for locations in [XXXXXX], for which landscape map is accessible from OpenStreetMap. 
The whole region is conformed as a [x × Y] regular grid. The height of each building is set as [X] meters. 
Each transmitter contains three antennas (model [720842A2]) with height as [35] meters and initial power as [46.00] dBm. 
The frequency used for each antenna is [2625] MHz. 

[Example图片 Environment Map] [Example图片 Radiomap]

1 sample is in the folder /fine-resolution.

## Acknowledgement
Please acknowledge the following paper if the dataset is useful for your research

@article{zhang2023radiomap,<br/>
  title={Radiomap Inpainting for Restricted Areas based on Propagation Priority and Depth Map}, <br/>
  author={Zhang, Songyang and Yu, Tianhang and Choi, Brian and Ouyang, Feng and Ding, Zhi},<br/>
  journal={arXiv preprint arXiv:2305.15526}, <br/>
  year={2023} <br/>
}

## Contact
Please contact Tianhang Yu for more information regarding the BRAT-Lab Dataset if you have any questions.

Email: thgyu@ucdavis.edu [You can also use your personal email address if you want]
