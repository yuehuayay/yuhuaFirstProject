clc
clear all;
close all;

c = 3e8;
RF.rangbinum = 128;
RF.chirpnum = 192;
RF.rfBandwidthMHz = 200;
RF.rfstartFreqMHz = 24000;
RF.frame_perodMs = 80;
RF.numchirpsPerFrame = 192;
RF.chirpSweepTimeUs = 130;
RF.chirpSweepDelayTimeUs = 60;
RF.FsMHz = 1.98;
RF.txAntNum = 2;
RF.rxAntNum = 4;
RF.NumChannel = RF.txAntNum*RF.rxAntNum;
obj.setVelocity =37;
obj.setDistance = 5;
obj.thetaAzi = 35;
obj.thetaEle = 0;
obj.thetaFov = -60:1:60;
lamda = c/((2*RF.rfstartFreqMHz*1e6 + RF.rfBandwidthMHz*1e6)/2);
% AntPos = [0,lamda/2,lamda,1.5*lamda,2*lamda,2.5*lamda,3*lamda,3.5*lamda];

Txpos = [0 0; 1 0];
Rxpos = [0 1; 1 -1; 2 0; 4 0];

Mimo_AntPos_TX1 = Txpos(1,:) + Rxpos;
Mimo_AntPos_TX2 = Txpos(2,:) + Rxpos;
Mimo_AntPos_TX1_lamda = (Txpos(1,:) + Rxpos)*lamda/2;
Mimo_AntPos_TX2_lamda = (Txpos(2,:) + Rxpos)*lamda/2;

Mimo_AntPos_ALL = cat(1,Mimo_AntPos_TX1,Mimo_AntPos_TX2);
Mimo_AntPos_ALL_lamda = Mimo_AntPos_ALL*lamda/2;
figure(1);
plot(Mimo_AntPos_ALL(:,1),Mimo_AntPos_ALL(:,2),'.', 'MarkerSize', 20);
title("mimo阵列");

for i = 1:length(Mimo_AntPos_ALL_lamda(:,1)) 
    text(Mimo_AntPos_ALL(i,1) + 0.1, Mimo_AntPos_ALL(i,2)+0.1,num2str(i),...
        'FontSize',10,'Color','red','HorizontalAlignment','left');
end

N = 4;NumAzimFFTbins = 256;
T = RF.chirpSweepTimeUs*1e-6;
t = T/RF.rangbinum;
S = RF.rfBandwidthMHz*1e6/T;
% idle = T*8/7 - T; 
idle = RF.chirpSweepDelayTimeUs*1e-6;
T12 = T + idle;
Tr2r = T + idle + T;
Tr3r = T + idle + T + T;
Doppler_chirp = RF.chirpnum/3;
Vmax = lamda/(2*Tr3r);
Vres = Vmax/(RF.chirpnum/3);
dres = c/(2*RF.rfBandwidthMHz*1e6);
Ares = ((2/N)*180/pi)/(NumAzimFFTbins/N);
t1  = 0:t:t*(RF.rangbinum -1);
% chirp_idx = 0:chirpCfg.numChirps-1;
% 
% 1. range phasor
% range_phasor = A*exp(1i*((2*pi*f*t) + phi_0));

Amodd = zeros(RF.rangbinum,RF.chirpnum/3,RF.rxAntNum);
Ameven = zeros(RF.rangbinum,RF.chirpnum/3,RF.rxAntNum);



%%%chirp_delay_IF信号生成%%%%%%TX1_1
for Rangidx = 1:1:RF.rangbinum     
 for Dopidx = 1:1:Doppler_chirp
     for Rx_i = 1:1:length(Mimo_AntPos_TX1_lamda(:,1))
 NO = Dopidx-1;
 Amodd(Rangidx,Dopidx,Rx_i) =  exp(j*(2*pi*(S*2*obj.setDistance/c)*t1(Rangidx) + (4*pi*obj.setVelocity*Tr3r*NO)/lamda ...
     + (2*pi*Mimo_AntPos_TX1_lamda(Rx_i,1)*sind(obj.thetaAzi)*cosd(obj.thetaEle))/lamda + (2*pi*Mimo_AntPos_TX1_lamda(Rx_i,2)*sind(obj.thetaEle))/lamda));
     end
 end
end

%%%chirp_delay_IF信号生成%%%%%%TX1_2
for Rangidx = 1:1:RF.rangbinum     
 for Dopidx = 1:1:Doppler_chirp
     for Rx_i = 1:1:length(Mimo_AntPos_TX1_lamda(:,1))
 NO = Dopidx-1;
 Amodd2(Rangidx,Dopidx,Rx_i) =  exp(j*(2*pi*(S*2*obj.setDistance/c)*t1(Rangidx) + (4*pi*obj.setVelocity*(Tr3r*NO + T12))/lamda ...
     + (2*pi*Mimo_AntPos_TX1_lamda(Rx_i,1)*sind(obj.thetaAzi)*cosd(obj.thetaEle))/lamda + (2*pi*Mimo_AntPos_TX1_lamda(Rx_i,2)*sind(obj.thetaEle))/lamda));
     end
 end
end

%%%chirp_delay_IF信号生成%%%%%%TX2
for Rangidx = 1:1:RF.rangbinum
 for Dopidx = 1:1:Doppler_chirp
     for Rx_i = 1:1:length(Mimo_AntPos_TX2_lamda(:,1))
 NO = Dopidx-1;
 Ameven(Rangidx,Dopidx,Rx_i) = exp(j*(2*pi*(S*2*obj.setDistance/c)*t1(Rangidx) + (4*pi*obj.setVelocity*(Tr3r*NO + Tr2r))/lamda ...
     + (2*pi*Mimo_AntPos_TX2_lamda(Rx_i,1)*sind(obj.thetaAzi) *cosd(obj.thetaEle))/lamda + (2*pi*Mimo_AntPos_TX2_lamda(Rx_i,2)*sind(obj.thetaEle))/lamda));
     end
 end
end

Am_ALL = cat(3,Amodd,Ameven);

Am_ALL = Am_ALL+randn(size(Am_ALL));
Amodd = Amodd+randn(size(Amodd));
Ameven = Ameven+randn(size(Ameven));

%check adc data
figure(2);
subplot(121);
plot(real(Am_ALL(:,32,1)));
title("ADCdatachannel1");
subplot(122);
plot(real(Am_ALL(:,32,2)));
title("ADCdatachannel2");
% figure(3);
% plot(db(fft((testdata_complex(:,20) - mean(testdata_complex(:,20))),128,1)));
% 
% figure(4);
% plot(db(fft((testdata_complex2(:,20) - mean(testdata_complex2(:,20))),128,1)));

%%window
windowBlackman = blackman(RF.rangbinum); 
windowBlackmanRange2D = repmat(windowBlackman,1,Doppler_chirp);
windowBlackmanRange3D = repmat(windowBlackmanRange2D,[1,1,RF.NumChannel]);

windowBlackmanRange3D1 = repmat(windowBlackmanRange2D,[1,1,4]);


Am_ALLRangefft =  fft(Am_ALL.*windowBlackmanRange3D,RF.rangbinum,1);
% AmoddRangefft = fft(Amodd.*windowBlackmanRange3D,RF.rangbinum,1);
% AmeveRangefft = fft(Ameven.*windowBlackmanRange3D,RF.rangbinum,1);
Amodd2_Range = fft(Amodd2.*windowBlackmanRange3D1,RF.rangbinum,1);


Am_ALLDopplerfft = fft(Am_ALLRangefft,RF.chirpnum/3,2);
% AmoddDopplerfft = fft(AmoddRangefft,RF.chirpnum/2,2);
% AmeveDopplerfft = fft(AmeveRangefft,RF.chirpnum/2,2);
Amodd2_Doppler = fft(Amodd2_Range.*windowBlackmanRange3D1,RF.chirpnum/3,2);

% figure(3);
% RDM = db(squeeze(Am_ALLDopplerfft(:,:,1)));
% mesh(RDM);
% title("RDM");
% Amodd_doppler = abs(fft(Amodd,half_chirp,2));
% Ameven_doppler =  abs(fft(Ameven,half_chirp,2));


%非相干累加
Dopplerfft_all = zeros(RF.rangbinum,Doppler_chirp);
for i_rx= 1:1:RF.NumChannel
Dopplerfft_all = Dopplerfft_all+abs(Am_ALLDopplerfft(:,:,i_rx));
end
figure(3)
RDM = db(Dopplerfft_all);
mesh(RDM);
title("RDM");


%%计算目标的idx，实际应选用cfar的结果，此为计算方便
Rangbinid = round(obj.setDistance/dres)+1;
Dopplerbinid = round(obj.setVelocity/Vres)+1;
Dopplerbinid = Dopplerbinid-floor((Dopplerbinid/Doppler_chirp))*Doppler_chirp;


%%%%%%%%取出8个Rx的通道数据Samples%%%%%%%%%%%%%
for i_channelidx = 1:1:RF.NumChannel
ChannleSamples(i_channelidx) = Am_ALLDopplerfft(Rangbinid,Dopplerbinid,i_channelidx);
end

for i_testidx = 1:1:4
TX1_2channelsamples(i_testidx) = Amodd2_Doppler(Rangbinid,Dopplerbinid,i_testidx);
end
 

%%%%%%%%%%%%%%%速度解模糊%%%%%%%%%%%%%
datacomplexTX1 = ChannleSamples(1);  datacomplexTX2 = TX1_2channelsamples(1);
Deltphi = atan2(imag(datacomplexTX2/datacomplexTX1), real(datacomplexTX2/datacomplexTX1));
q = -2:1:2;
V = Vres*(Dopplerbinid-1) + q*Vmax;
V_Deltphi = 4*pi*V*T12/lamda;
weizhi_V_Deltphi=cos(V_Deltphi) + j*sin(V_Deltphi);
weizhi_Deltphi = cos(Deltphi) + j*sin(Deltphi);
% weizhi_Deltphi = 
figure(4);

compass(weizhi_V_Deltphi);hold on
compass(weizhi_Deltphi, "-r");

for i=1:1:5
text(real(weizhi_V_Deltphi(i)),imag(weizhi_V_Deltphi(i)),num2str(V(i)))
end
 kk =1;

%%%%%%%%%%%%%%%速度补偿%%%%%%%%%%%%%
V_deltaphi = 4*pi*obj.setVelocity*Tr2r/lamda;
ChannleSamples(5:8) = ChannleSamples(5:8)*exp(-j*V_deltaphi);


%%%%%%%%%%%进行通道重排%%%%%%%%%%%%%上次错误是由于信号生成的时候天线阵列忘记乘以lamda/2了%%
Array_Mimo = [1 5 2 6 3 7 4 8];
for Azimidx = 1:1:RF.NumChannel
ChannleSamples_Array(Azimidx) = ChannleSamples(Array_Mimo(Azimidx));
end


%%%%%%%%%%%CHECK DELTAPHASE%%%%%%%%%%
phase = angle(ChannleSamples_Array);
 phase_diff = diff(phase);
 for angleidx = 1:RF.NumChannel-1
    if phase_diff(angleidx) > pi
        phase_diff(angleidx) = phase_diff(angleidx)-2*pi;
    end
    if  phase_diff(angleidx) < -pi 
         phase_diff(angleidx) = phase_diff(angleidx) + 2*pi;  
    end
 end




%%%%%%%%%%%%%%%FFT测角%%%%%%%%%%%%%%

ChannleSamples_Azim = fft(ChannleSamples_Array(5:8),NumAzimFFTbins);
ChannleSamples_Azim_dB = db(fft(ChannleSamples_Array(5:8),NumAzimFFTbins));
figure(5);
plot(ChannleSamples_Azim_dB);
title("Angle-fft")

[Power,los] = max(ChannleSamples_Azim_dB);
if (los > NumAzimFFTbins/2)
    los = los - NumAzimFFTbins;
end
Angle_result = asind(2*los/NumAzimFFTbins); %%公式 sin(teta) = wxrad /pi ; wxrad = 2*pi*(peakidx/Azimbins) 
%%结果正确
kk=2;


%%%%DBF测角仿真%%%%%%%
% for i_Rx=1:1:length(AntPos)
%     for i_Angle = 1:1:length(obj.thetaFov)
% SteringVector(i_Rx,i_Angle) = exp(-j*((2*pi*AntPos(i_Rx)*sind(obj.thetaFov(i_Angle)))/lamda));
%     end
% end
% 
% % RX = [channeldataRx1 channeldataRx2 channeldataRx3 channeldataRx4 channeldataRx5 channeldataRx6 channeldataRx7 channeldataRx8];
% RX = [channeldataRx1 channeldataRx2];
% RXdeltaphasecomplex = channeldataRx2/channeldataRx1;
%     for i_theta = 1:1:length(obj.thetaFov)
% SteringVector1phase1(i_theta) = exp(-j*pi*sind(obj.thetaFov(i_theta)));
%     end
% 
%     for i_theta2 = 1:1:length(obj.thetaFov)
%     SteringVector1phase2(i_theta2) = exp(-j*((2*pi*AntPos(1)*sind(obj.thetaFov(i_Angle)))/lamda));
%     end
% 
%    SteringVector1phase_test =  SteringVector1phase2 - SteringVector1phase1;
%   DBF_result = SteringVector1phase_test*RXdeltaphasecomplex;
%    DBF_resultdbf = RX*SteringVector;
%   figure(2);
%   plot(abs(DBF_result));
% kkk =1;                     %%%结果正确


%%%%%%原始数据生成.bin%%%%%%%%%%%%%%%%%
DestFilenameodd = fopen('destodddata.bin',"ab");
DestFilenameeven = fopen('destevendata.bin',"ab");
for i = 1:1:RF.rxAntNum
    for j = 1:1:RF.chirpnum/2
      for k = 1:1:RF.rangbinum
        fwrite(DestFilenameodd,real(Amodd(k,j,i)),'float');
        fwrite(DestFilenameodd,imag(Amodd(k,j,i)),'float');
      end
    end
end                                                     %%奇数chirp

for i = 1:1:RF.rxAntNum
    for j = 1:1:RF.chirpnum/2
      for k = 1:1:RF.rangbinum
        fwrite(DestFilenameeven,real(Ameven(k,j,i)),'float');
        fwrite(DestFilenameeven,imag(Ameven(k,j,i)),'float');
      end
    end
end                                                    %%偶数chirp
kk =1; 

%%%%%%原始数据bin文件读取验证%%%%%%%%%%%%%%%%%
fileNametodd =fopen('destodddata.bin','rb');
ReadDatalengthallodd = RF.rangbinum * RF.chirpnum/2 *RF.rxAntNum*2;
testdataodd = fread(fileNametodd,ReadDatalengthallodd,"float");
testdataoddcomplex = testdataodd(1:2:end-1)+(i*testdataodd(2:2:end));

testdataodd = reshape(testdataoddcomplex,RF.rangbinum,RF.chirpnum/2,2);
figure(1);
plot(testdataodd(:,1));
restdataoddRangefft = fft(testdataodd.*windowBlackmanRange3D,RF.rangbinum,1);
restdataoddDopplerfft = fft(restdataoddRangefft,RF.chirpnum/2,2);
figure(2);
mesh(abs(squeeze(restdataoddDopplerfft(:,:,1))));

fileNameteven =fopen('destevendata.bin','rb');
ReadDatalengthalleven = RF.rangbinum * RF.chirpnum/2 *RF.rxAntNum*2;
testdataeven = fread(fileNameteven,ReadDatalengthalleven,"float");
fclose("all");
kkk =1;
%%%%%%%%%%%%%%%%速度解模糊%%%%%%%%%%%%%
% figure(1);
% plot(Amodd_doppler); hold on
% plot(Ameven_doppler);
% 
% [magodd,idxodd] = max(Amodd_doppler);
% [mageven,idxeven] = max(Ameven_doppler);
% % ssS_v = 
% Amoddpeak = Amodddopplerfft(idxodd-1);
% Amevenpeak = Amevenopplerfft(idxeven-1);
% Deltphi = atan2(imag(Amevenpeak/Amoddpeak), real(Amevenpeak/Amoddpeak));
% 
%  % test = Amevenpeak/Amoddpeak;
% 
% q = -5:1:5;
% V = vres*(idxodd-1) + q*Vmax;
% V_Deltphi = 4*pi*V*T12/lamda;
% weizhi_V_Deltphi=cos(V_Deltphi) + j*sin(V_Deltphi);
% weizhi_Deltphi = cos(Deltphi) + j*sin(Deltphi);
% % weizhi_Deltphi = 
% figure(2);
% 
% compass(weizhi_V_Deltphi);hold on
% compass(Amevenpeak/Amoddpeak, "-r");
% 
% for i=1:1:11
% text(real(weizhi_V_Deltphi(i)),imag(weizhi_V_Deltphi(i)),num2str(V(i)))
% end
%  kk =1;
% 
% %%%%%%%%专利公式仿真%%%%%%
% 
%   testsituation = abs(Amoddpeak.*exp(j*V_Deltphi) - Amevenpeak);
%  %testsituation = abs(conj((Amevenpeak ./ Amoddpeak))*exp(j*V_Deltphi));
%  kkk=1;
%  figure(3);
%  plot(testsituation);












