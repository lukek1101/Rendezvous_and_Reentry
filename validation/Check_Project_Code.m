function results = Check_Project_Code()
%CHECK_PROJECT_CODE Analyze all active MATLAB source and validation packages.
% Legacy snapshots and temporary experiment scripts are intentionally excluded.
    root = fileparts(fileparts(mfilename('fullpath')));
    entries = dir(fullfile(root, '*.m'));
    packages = dir(fullfile(root, '+*'));
    folders = [{'validation'}, {packages([packages.isdir]).name}];
    for k = 1:numel(folders)
        entries = [entries; dir(fullfile(root, folders{k}, '**', '*.m'))]; %#ok<AGROW>
    end
    issue_count = 0;
    for k = 1:numel(entries)
        file = fullfile(entries(k).folder, entries(k).name);
        issues = checkcode(file, '-id');
        issue_count = issue_count + numel(issues);
        for j = 1:numel(issues)
            fprintf('%s:%d [%s] %s\n', file, issues(j).line, issues(j).id, issues(j).message);
        end
    end
    results = struct('file_count', numel(entries), 'issue_count', issue_count, 'passed', issue_count == 0);
    assert(results.passed, 'MATLAB Code Analyzer reported %d issue(s).', issue_count);
    fprintf('Project Code Analyzer: PASS (%d files)\n', results.file_count);
end
