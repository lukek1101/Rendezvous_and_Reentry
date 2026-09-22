function cfg = load_optimizer_config(sys, run_cfg)
    cfg = struct();
    if nargin < 1
        sys = struct();
    end
    if nargin < 2
        run_cfg = struct();
    end

    config_path = "";
    selection_msg = "";
    allow_env = get_run_bool_field(run_cfg, {'runtime','allow_environment_overrides'}, true);
    if allow_env
        config_path = strtrim(string(getenv('RENDEZVOUS_CONFIG_JSON')));
        if strlength(config_path) == 0
            config_path = strtrim(string(getenv('RENDEZVOUS_CONFIG')));
        end
        if strlength(config_path) > 0
            selection_msg = "explicit RENDEZVOUS_CONFIG_JSON";
        end
    end

    if strlength(config_path) == 0
        python_mode = upper(get_run_string_field(run_cfg, {'python_config','mode'}, "AUTO"));
        if python_mode == "NONE" || python_mode == "OFF" || python_mode == "DISABLED"
            fprintf('Python optimizer JSON disabled by Mission_Run_Config.m. Using MATLAB/run-control defaults.\n');
            return;
        elseif python_mode == "FILE"
            config_path = get_run_string_field(run_cfg, {'python_config','file'}, "");
            selection_msg = "Mission_Run_Config.m python_config.file";
        else
            [config_path, selection_msg] = select_python_config_from_index(sys, run_cfg);
        end
    end

    if strlength(config_path) == 0
        fprintf('No mission JSON config selected. Using MATLAB defaults.\n');
        return;
    end

    if ~is_absolute_path(config_path)
        config_path = string(fullfile(mission.project_root(), char(config_path)));
    end
    if ~isfile(config_path)
        error('Mission JSON config not found: %s', char(config_path));
    end

    cfg = jsondecode(fileread(config_path));
    cfg.import_record=struct('path',config_path,'sha256',mission.file_sha256(config_path), ...
        'selection',selection_msg);
    fprintf('Loaded mission JSON config: %s (%s)\n', char(config_path), char(selection_msg));
    if has_json_path(cfg, {'archive','case_id'})
        fprintf('   config case_id: %s\n', char(get_json_string(cfg, {'archive','case_id'}, "")));
        fprintf('   settings_hash : %s\n', char(get_json_string(cfg, {'archive','settings_hash'}, "")));
    end
end

function [config_path, selection_msg] = select_python_config_from_index(sys, run_cfg)
    config_path = "";
    selection_msg = "";
    if nargin < 2
        run_cfg = struct();
    end

    root_dir = mission.project_root();
    config_dir = fullfile(root_dir, 'configs');
    index_path = fullfile(config_dir, 'python_solution_index.json');
    if ~isfile(index_path)
        latest_path = fullfile(config_dir, 'latest_python_solution.json');
        if isfile(latest_path)
            config_path = string(latest_path);
            selection_msg = "fallback latest_python_solution.json";
        end
        return;
    end

    index_cfg = jsondecode(fileread(index_path));
    if ~isfield(index_cfg, 'cases') || isempty(index_cfg.cases)
        return;
    end

    cases = index_cfg.cases;
    allow_env = get_run_bool_field(run_cfg, {'runtime','allow_environment_overrides'}, true);
    python_mode = upper(get_run_string_field(run_cfg, {'python_config','mode'}, "AUTO"));

    desired_case_id = "";
    desired_hash = "";
    if python_mode == "CASE_ID"
        desired_case_id = strtrim(get_run_string_field(run_cfg, {'python_config','case_id'}, ""));
    elseif python_mode == "HASH"
        desired_hash = strtrim(get_run_string_field(run_cfg, {'python_config','hash'}, ""));
    end
    if allow_env
        desired_case_id_env = strtrim(string(getenv('RENDEZVOUS_CONFIG_CASE_ID')));
        desired_hash_env = strtrim(string(getenv('RENDEZVOUS_CONFIG_HASH')));
        if strlength(desired_case_id_env) > 0
            desired_case_id = desired_case_id_env;
        end
        if strlength(desired_hash_env) > 0
            desired_hash = desired_hash_env;
        end
    end

    desired_burn = desired_burn_model_for_index(sys, run_cfg);
    [desired_drag_enabled, desired_drag_model] = desired_drag_for_index(sys, run_cfg);

    best_score = -inf;
    best_path = "";
    best_case_id = "";

    for ii = 1:numel(cases)
        entry = cases(ii);
        entry_case_id = get_json_string(entry, {'case_id'}, "");
        entry_hash = get_json_string(entry, {'settings_hash'}, "");

        if strlength(desired_case_id) > 0
            if entry_case_id == desired_case_id
                [config_path, ~] = config_path_from_index_entry(config_dir, entry);
                selection_msg = "case id " + desired_case_id;
                return;
            end
            continue;
        end

        if strlength(desired_hash) > 0 && entry_hash ~= desired_hash
            continue;
        end

        entry_burn = upper(get_json_string(entry, {'phase1','burn_model'}, ""));
        if strlength(desired_burn) > 0 && entry_burn ~= desired_burn
            continue;
        end

        score = 0;
        if entry_burn == desired_burn
            score = score + 100;
        end
        score = score + scenario_match_score(sys, entry);

        entry_drag_enabled = get_json_bool(entry, {'environment','atmospheric_drag_enabled'}, false);
        entry_drag_model = upper(get_json_string(entry, {'environment','atmospheric_drag_model'}, "ISA76"));
        if entry_drag_enabled == desired_drag_enabled
            score = score + 20;
        end
        if entry_drag_model == desired_drag_model
            score = score + 5;
        end
        if strlength(desired_hash) > 0
            score = score + 1000;
        end

        if score >= best_score
            [candidate_path, ~] = config_path_from_index_entry(config_dir, entry);
            best_score = score;
            best_path = candidate_path;
            best_case_id = entry_case_id;
        end
    end

    if strlength(best_path) > 0
        config_path = best_path;
        selection_msg = sprintf("auto index match case %s", best_case_id);
    elseif python_mode == "CASE_ID" && strlength(desired_case_id) > 0
        error('Requested Python config case_id was not found: %s', char(desired_case_id));
    elseif python_mode == "HASH" && strlength(desired_hash) > 0
        error('Requested Python config settings_hash was not found: %s', char(desired_hash));
    end
end

function desired_burn = desired_burn_model_for_index(sys, run_cfg)
    desired_burn = "";
    if nargin >= 2
        desired_burn = strtrim(get_run_string_field(run_cfg, {'maneuver','burn_model'}, ""));
    end
    if nargin >= 2 && get_run_bool_field(run_cfg, {'runtime','allow_environment_overrides'}, true)
        burn_env = strtrim(string(getenv('RENDEZVOUS_BURN_MODEL')));
        if strlength(burn_env) > 0
            desired_burn = burn_env;
        end
    end
    if strlength(desired_burn) == 0 && isfield(sys, 'maneuver') && isfield(sys.maneuver, 'default_burn_model')
        desired_burn = string(sys.maneuver.default_burn_model);
    end
    desired_burn = normalize_burn_model_label(desired_burn);
end

function model = normalize_burn_model_label(value)
    model = upper(strtrim(string(value)));
    if model == "FINITE" || model == "FINITE_BURN" || model == "FINITE_IMPULSE"
        model = "FINITE_BURN";
    elseif model == "IMPULSIVE" || model == "INSTANT" || model == "INSTANTANEOUS" || model == "CUSTOM_IMPULSE"
        model = "IMPULSIVE";
    end
end

function [enabled, model] = desired_drag_for_index(sys, run_cfg)
    if nargin >= 2 && drag_starts_after_berthing(run_cfg)
        enabled = false;
        model = "ISA76";
        return;
    end

    drag_env = "";
    if nargin >= 2 && get_run_bool_field(run_cfg, {'runtime','allow_environment_overrides'}, true)
        drag_env = upper(strtrim(string(getenv('RENDEZVOUS_ATMOSPHERIC_DRAG'))));
    end
    if strlength(drag_env) > 0
        enabled = ~(drag_env == "OFF" || drag_env == "NONE" || drag_env == "0" || drag_env == "DISABLED");
        if enabled
            model = drag_env;
        else
            model = "ISA76";
        end
        return;
    end

    enabled = false;
    model = "ISA76";
    if isfield(sys, 'environment') && isfield(sys.environment, 'atmospheric_drag')
        drag = sys.environment.atmospheric_drag;
        if isfield(drag, 'enabled')
            enabled = logical(drag.enabled);
        end
        if isfield(drag, 'model')
            model = upper(string(drag.model));
        end
    end
end

function score = scenario_match_score(sys, entry)
    score = 0;
    score = score + numeric_match_score(sys.h_insert/1000, get_json_number(entry, {'scenario','h_chaser_km'}, NaN), 1e-6, 20);
    score = score + numeric_match_score(sys.h_target/1000, get_json_number(entry, {'scenario','h_target_km'}, NaN), 1e-6, 20);
end

function score = numeric_match_score(a, b, tol, points)
    if isfinite(a) && isfinite(b) && abs(a - b) <= tol
        score = points;
    else
        score = 0;
    end
end

function [config_path, selection_msg] = config_path_from_index_entry(config_dir, entry)
    rel_path = get_json_string(entry, {'path'}, "");
    if strlength(rel_path) == 0
        config_path = "";
        selection_msg = "";
        return;
    end

    if is_absolute_path(rel_path)
        config_path = rel_path;
    else
        rel_path = strrep(rel_path, "/", filesep);
        config_path = string(fullfile(config_dir, char(rel_path)));
    end
    selection_msg = get_json_string(entry, {'case_id'}, "");
end
