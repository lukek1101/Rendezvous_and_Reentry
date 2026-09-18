function tf = drag_starts_after_berthing(run_cfg)
    scope = upper(get_run_string_field(run_cfg, {'environment','atmospheric_drag','apply_from_phase'}, "PHASE1"));
    tf = scope == "PHASE3" || scope == "POST_BERTHING" || scope == "AFTER_BERTHING";
end
