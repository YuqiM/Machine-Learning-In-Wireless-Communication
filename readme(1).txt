-----------CIS-online HW1 By Patrick(YuqiMeng) under professor Danijela Cabric at UCLA-------------------
The code performs the simulation of the transmission of BPSK
QPSK,4PAM,16QAM signal with different properties and specifications.
-------------------On how to use the code---------------------------------
The successful execution of the code requires all the functions included in this package that is 'BERestimate.m','qam16recombine.m',
'qam16tobit.m','qammap16.m','tospec.m'
The main code are in 4 parts for 4 types of signal:
'CIShomework1_4PAM.m'
'CIShomework1_16QAM.m'
'CIShomework1_BPSK.m'
'CIShomework1_QPSK.m'

And some functions which are not self defined and are in the matlab 2016a communcation toolbox are also used.
Such as comm.PAMModulator,comm.PAMDemodulator,comm.BPSKModulator,comm.BPSKDemodulator,ammod,amdemod,comm.ConstellationDiagram.
Then you can just load all the '.m' files onto your matlab and click run to view all the graphs , all the code is annotated and the output BER is calculated for the 15dB AWGN channel
selected parameters in the code can be changed to produce different results.
More specific description of the coding process is also described in paragraph at the front of the main code.
All the simulation pictures are in the 'simulation results' folder.

