% Bounded regression only; no optimization or archive regeneration.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root); addpath(fullfile(root,'validation'));
folder=fileparts(mfilename('fullpath'));
diary(fullfile(folder,'matlab_validation.log'));
cleanup=onCleanup(@()diary('off'));
results=Run_All_Validations();
save(fullfile(folder,'verification.mat'),'results');
fid=fopen(fullfile(folder,'summary.json'),'w');
fprintf(fid,'%s\n',jsonencode(struct('passed',results.passed, ...
    'reference_profiles',results.reference_profiles),PrettyPrint=true));
fclose(fid);
