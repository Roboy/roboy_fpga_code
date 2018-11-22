Fs      = 37000;
F3db    =   100;
filtdes = fdesign.lowpass('n,f3db', 2, F3db, Fs);
butterFilter      = design(filtdes,'butter',...
    'SystemObject',true,'FilterStructure','df1sos');
fvtool(butterFilter, 'Fs', Fs, 'FrequencyScale', 'log');

butterFilter.NumeratorCoefficientsDataType         = 'Custom';
butterFilter.CustomNumeratorCoefficientsDataType   = numerictype([],16);
butterFilter.CustomDenominatorCoefficientsDataType = numerictype([],16);
butterFilter.CustomScaleValuesDataType             = numerictype([],16);
butterFilter.SectionInputDataType                  = 'Custom';
butterFilter.CustomSectionInputDataType            = numerictype([],20,15);
butterFilter.SectionOutputDataType                 = 'Custom';
butterFilter.CustomSectionOutputDataType           = numerictype([],20,15);
butterFilter.NumeratorProductDataType              = 'Full precision';
butterFilter.DenominatorProductDataType            = 'Full precision';
butterFilter.NumeratorAccumulatorDataType          = 'Custom';
butterFilter.CustomNumeratorAccumulatorDataType    = numerictype([],32,24);
butterFilter.DenominatorAccumulatorDataType        = 'Custom';
butterFilter.CustomDenominatorAccumulatorDataType  = numerictype([],32,25);
butterFilter.OutputDataType                        = 'Custom';
butterFilter.CustomOutputDataType                  = numerictype([],12,0);
butterFilter.RoundingMethod                        = 'nearest';
butterFilter.OverflowAction                        = 'wrap';

fvtool(butterFilter, 'Fs', Fs, 'FrequencyScale', 'log','Arithmetic','fixed');

scale(butterFilter,'Linf');
scaless = butterFilter.ScaleValues;
disp(scaless);

workingdir = tempname;

userstim = [];
for n = [50, 100, 150, 200, 250, 300]
  userstim = [userstim, sin(2*pi*n/Fs*(0:Fs/n))]; %#ok
end

generatehdl(butterFilter, 'Name', 'hdlbutter',...
                 'TargetLanguage', 'Verilog',...
                 'TargetDirectory', workingdir, ...
                 'GenerateHDLTestbench', 'on', ...
                 'TestBenchUserStimulus', userstim, ...
                 'InputDataType',numerictype(1,12,0));