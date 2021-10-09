% =========================================================================
% Liadr TX Trig and RX Data Simulation
% =========================================================================
% Author : Lin Junyang
% Date   : 2021.10
% Version: 01
% -------------------------------------------------------------------------
% Lidar  TX/RX ◖ ←-----------------------→  ❒ Object
% -------------------------------------------------------------------------
% Operation Sequency:
%  1. Lidar generates internal laser firing triger signal.
%  2. Lidar TX sends laser pulse out.Reference signal returns immediately.
%  3. Laser echo returns to Lidar and convert to data by ADC.
% -------------------------------------------------------------------------
% This simulation simulates the Lidar trigger signal and the received data.
% Simulation of 3 targets echo signals.
% =========================================================================



clear; clc; close all

c = physconst('LightSpeed');
fs = 1e9;  % 1GHz sample rate
T = 2e-6;  % second, laser pulse output period
ref_dist = 0.3;  % meter
target_dist = 45+randi(10);  % meter
target2_dist = 70+randi(10);  % meter
target3_dist = 95+randi(10);  % meter
laser_out_delay = 4e-9;  % second
laser_pulse_width = 5e-9;

adc_bwidth = 9;
ref_peak = 20;
target1_peak = 40;
target2_peak = 32;
target3_peak = 22;

return_shape = [0.1 0.65 1 0.9 0.7 0.5 0.35 0.2 0.1 0 ...
    -0.1 -0.2 -0.25 -0.24 -0.22 -0.19 -0.15 -0.13 -0.1 -0.05 -0.04 -0.03 -0.02 -0.01 -0.005];
return_shape = return_shape/sum(return_shape);
% plot(return_shape)  % return signal shape of a single target.

%%
N = T*fs;
Wref = laser_pulse_width*fs;
Wtarget = Wref + 2;

ref_idx = floor((laser_out_delay + ref_dist*2/c)*fs + 0.5);
target_idx = floor((laser_out_delay + target_dist*2/c)*fs + 0.5);
target2_idx = floor((laser_out_delay + target2_dist*2/c)*fs + 0.5);
target3_idx = floor((laser_out_delay + target3_dist*2/c)*fs + 0.5);

sig = [];
trig = [];
for m = 1:100
    ref_peak = ref_peak * (0.9+0.2*rand());
    target1_peak = target1_peak * (0.85+0.3*rand());
    target2_peak = target2_peak * (0.85+0.3*rand());
    target3_peak = target3_peak * (0.85+0.3*rand());
    
    sig0 = zeros(1, N);
    sig0(ref_idx:ref_idx+Wref-1) = sig0(ref_idx) + ref_peak;
    sig0(target_idx:target_idx+Wtarget-1) = sig0(target_idx) + target1_peak;
    sig0(target2_idx:target2_idx+Wtarget-1) = sig0(target2_idx) + target2_peak;
    sig0(target3_idx:target3_idx+Wtarget-1) = sig0(target3_idx) + target3_peak;

    sig1 = zeros(1, N);
    sig1 = awgn_m(sig1, 5);  % awgn
    sig1 = sig1 + 2^(adc_bwidth-1);  %offset
    for k = 1:size(return_shape,2)
        sig1 = sig1 + awgn_m([zeros(1,k-1) sig0(1:N-k+1)]*return_shape(k), 10);  % awgn
    end

    sig1 = floor(sig1+0.5);
    sig1(sig1<0) = 0;
    sig1(sig1>511) = 511;

    trig1 = sig1*0;
    trig1(1) = 1;
    
    % set no signal cycle, simulation of no signal cycle. Comment out if not needed.
% %     if mod(m,16)==0
% %         sig1 = sig1*0 + 2^(adc_bwidth-1);
% %     end

    sig = [sig sig1];
    trig = [trig trig1];
end

subplot(221); plot(sig(1:21000)); title('ADC DATA');
subplot(223); plot(trig(1:21000)); ylim([-1 2]); title('TRIG');
subplot(222); plot(sig(1900:2900)); title('ADC DATA - echo sample');
subplot(224); plot(trig(1900:2900)); ylim([-1 2]); title('TRIG - sample');
%%













