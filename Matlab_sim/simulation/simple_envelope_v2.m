function [env] = simple_envelope(sig, fs, ds)
% https://www.mathworks.com/help/dsp/ug/envelope-detection.html
% sig is the signal array we are finding an envelope for
% fs is the sampling frequency
% ds is the downsampling

sig_sq = 2 * sig .* sig;

sig_ds = downsample(sig_sq, ds);

sig_lp = lowpass(sig_ds, 10, fs);

env = sqrt(sig_lp);

end

