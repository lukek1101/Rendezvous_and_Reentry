function sys=Legacy_Baseline_System()
%LEGACY_BASELINE_SYSTEM Frozen test data only; never an active vehicle menu.
root=fileparts(fileparts(mfilename('fullpath')));
frozen=load(fullfile(root,'docs','audit_baseline_2026-09-21','baseline.mat'),'result');
sys=frozen.result.config.system;
end
