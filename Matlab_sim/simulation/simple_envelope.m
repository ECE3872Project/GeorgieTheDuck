function [env] = simple_envelope(sig, fs, ds)
% https://www.mathworks.com/help/dsp/ug/envelope-detection.html

sig_sq = 2 * sig .* sig;

sig_ds = downsample(sig_sq, ds);

sig_lp = lowpass(sig_ds, 10, fs);

env = sqrt(sig_lp);

end

