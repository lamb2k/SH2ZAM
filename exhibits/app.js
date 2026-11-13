/* SH2ZAM interactive exhibit. MIT, Copyright (c) 2026 SH2ZAM Contributors.
 * Performance comes only from validated, recorded SH-2 model evidence.
 * The browser renders geometry and checks precision; it does not benchmark. */
'use strict';
(() => {
  const $ = id => document.getElementById(id), TAU = Math.PI * 2, MAX_WORLD_ERROR = 0.001;
  const reduced = matchMedia('(prefers-reduced-motion: reduce)');
  const canvases = [$('before-canvas'), $('after-canvas')];
  const contexts = canvases.map(canvas => canvas.getContext('2d'));
  const makeMesh = window.SH2ZAM_MESHES?.makeMesh;
  const specs = {
    saturn: ['Ring world', 'A low-poly planet and concentric rings, transformed in lockstep.'],
    cubes: ['Cube field', 'A lattice of independent cubes sharing one batched transform.'],
    torus: ['Twisted orbit', 'A continuous torus knot: more vertices, the same camera and math.']
  };
  const pathNames = {
    plain: 'Plain fixed-point C', reference: 'Software float', safe: 'SH2ZAM prior safe batch',
    prepared: 'SH2ZAM prepared C', mac: 'SH2ZAM native MAC.L', dmuls: 'SH2ZAM native DMULS.L'
  };
  const strategies = ['general-fallback', 'prepared-scalar', 'coordinate-reuse'];
  const modelPaths = ['plain', 'reference', 'prepared', 'mac', 'dmuls', 'safe'];
  let wasm, mesh, input, before, beforeBits, general, after, width = 1000, height = 440;
  let scene = 'saturn', detail = 3, baseline = 'plain', kernel = 'mac';
  let seconds = 0, playing = !reduced.matches, last = 0;
  let maxError = 0, maxWorldError = 0, maxGeneralWorldError = 0, fixedMismatches = 0;
  let outputChecksums = {}, outputsValid = false, visualError = null;
  let modelRow = null, modelData = null;
  let mathData = null, mathFamily = 'vector', mathOperation = 'vec3_normalize';
  let workloadData = null, workloadGroup = 'rotation', workloadProfile = 'bounded', workloadSize = 512;
  let workloadBaseline = 'strongest', workloadKernel = 'mac';
  const workloadSpecs = {
    rotation: {label: 'Quaternion rotation', sizes: [1, 8, 32, 128, 512], profiles: ['bounded', 'tiny', 'full-range', 'identity', 'zero'], paths: ['scalar', 'composed', 'composed_mac', 'batch', 'mac'], baselines: ['scalar', 'composed', 'composed_mac'], kernels: ['batch', 'mac']},
    spatial: {label: 'Sphere / box queries', sizes: [128, 512], profiles: ['sparse', 'mixed', 'dense', 'crowded', 'full-range'], paths: ['api_brute', 'inline_brute', 'inline_scatter', 'index'], baselines: ['api_brute', 'inline_brute', 'inline_scatter'], kernels: ['index']},
    normalize: {label: 'Vec3 normalization', sizes: [32], profiles: ['bounded', 'tiny', 'full-range'], paths: ['prior', 'current'], baselines: ['prior'], kernels: ['current']},
    atan2: {label: 'atan2', sizes: [32], profiles: ['bounded', 'tiny', 'full-range'], paths: ['prior', 'current'], baselines: ['prior'], kernels: ['current']},
    atan: {label: 'atan', sizes: [32], profiles: ['bounded', 'tiny', 'full-range'], paths: ['prior', 'current'], baselines: ['prior'], kernels: ['current']}
  };
  const workloadPathNames = {scalar: 'Prior quat_rotate · repeated calls', composed: 'Prior matrix + C batch', composed_mac: 'Prior matrix + existing MAC.L', batch: 'New quaternion batch', mac: 'New quaternion batch + optional MAC.L', api_brute: 'Prior predicate API loop', inline_brute: 'Independent C · packed output', inline_scatter: 'Independent C · sparse hit writes', index: 'New spatial index', prior: 'Prior SH2ZAM · 3374d77', current: 'Current SH2ZAM'};
  const workloadProfileNames = {bounded: 'Bounded', tiny: 'Tiny', 'full-range': 'Full range', identity: 'Identity quaternion', zero: 'Zero quaternion', sparse: 'Sparse', mixed: 'Mixed', dense: 'Dense / contained', crowded: 'Crowded / partial overlap'};
  const mathFamilies = {scalar: 'Scalar arithmetic', vector: 'Vectors', matrix: 'Matrices', quaternion: 'Quaternions', trig: 'Trigonometry', geometry: 'Geometry'};
  const mathProfiles = ['bounded', 'tiny', 'full-range'];
  const positive = value => Number.isSafeInteger(value) && value > 0;
  const number = value => value.toLocaleString('en-US');

  function publishVisualState() {
    window.SH2ZAM_EXHIBIT_STATE = {
      ready: !!(wasm && input && outputsValid), modelReady: !!modelRow, scene, detail,
      count: mesh ? mesh.vertices.length / 3 : modelRow?.vertices || 0,
      maxError, maxWorldError, maxGeneralWorldError, fixedMismatches, outputChecksums, outputsValid,
      preparedFastPath: !!(wasm && (wasm.candidate_available() & 1)),
      optimizedStrategy: wasm ? strategies[wasm.optimized_path()] : null,
      coordinateReuseBlockSize: wasm ? wasm.reuse_block_size() : 0,
      seconds, playing, error: visualError
    };
  }

  function setCodeSnippets() {
    $('baseline-code-title').textContent = baseline === 'plain'
      ? 'Without SH2ZAM · one fixed-point row' : 'Without SH2ZAM · one float32 row';
    $('baseline-code').textContent = baseline === 'plain'
      ? '// Bounds qualified for this matrix.\nint64_t sum =\n    (int64_t)row[0] * x\n  + (int64_t)row[1] * y\n  + (int64_t)row[2] * z;\n// SH-2 arithmetic shift floors once.\nout = (int32_t)((sum >> 16) + row[3]);'
      : 'float out =\n    row[0] * x\n  + row[1] * y\n  + row[2] * z\n  + row[3];\n\n// Compiled for SH-2:\n// software floating-point helpers.';
    $('optimized-code-title').textContent = kernel === 'mac'
      ? 'With SH2ZAM · optional native MAC.L' : 'With SH2ZAM · prepared batch';
    $('optimized-code').textContent = 'sh2z_mesh_view_t mesh;\nsh2z_transform_plan_t plan;\n\n'
      + '// Once for an immutable mesh:\nif (!sh2z_mesh_view_init(&mesh, vertices, count))\n    return false;\n\n'
      + '// Each new matrix:\nif (!sh2z_transform_plan_init(&plan, &mesh, &matrix))\n    return false;\n'
      + (kernel === 'mac' ? 'return sh2z_transform_plan_execute_mac(\n    out, &plan, 0, count);'
        : 'return sh2z_transform_plan_execute(\n    out, &plan);');
    $('optimized-source').href = 'https://github.com/lamb2k/SH2ZAM/blob/main/saturn/include/sh2zam/'
      + (kernel === 'mac' ? 'batch_experimental.h' : 'sh2zam.h');
    $('kernel-contract').textContent = kernel === 'mac'
      ? 'MAC.L is an optional experimental API. It uses native accumulation when bounds qualify and SR.S is clear; otherwise it uses the prepared fallback.'
      : 'Prepared C proves bounds for each new matrix and uses the exact full-range fallback when needed. The source mesh stays immutable; output is a separate buffer.';
  }

  function showModelResults() {
    const data = window.SH2ZAM_MODEL_RESULTS, run = data?.run;
    const supported = (data?.schema === 1 && data.kernelSet === 'float-safe-prepared-fused-sh2-model-v1')
      || (data?.schema === 2 && data.kernelSet === 'float-plain-fixed-safe-prepared-fused-sh2-model-v2');
    const row = Array.isArray(data?.records) ? data.records.find(record => record.scene === scene && record.detail === detail) : null;
    const vertices = mesh ? mesh.vertices.length / 3 : row?.vertices;
    const valid = supported && run?.environment === 'isolated-sh2-interpreter' && row
      && row.vertices === vertices && row.sampleCount === 21 && row.fixedMismatches === 0
      && Number.isFinite(row.maximumWorldError) && row.maximumWorldError >= 0
      && row.maximumWorldError <= 2 / 65536;
    modelRow = valid ? row : null;
    modelData = valid ? data : null;
    const beforeCycles = valid && positive(row.medianCycles?.[baseline]) ? row.medianCycles[baseline] : null;
    const afterCycles = valid && positive(row.medianCycles?.[kernel]) ? row.medianCycles[kernel] : null;
    const ready = beforeCycles !== null && afterCycles !== null;
    const savedPercent = ready ? 100 * (1 - afterCycles / beforeCycles) : null;
    const speedup = ready ? beforeCycles / afterCycles : null;
    window.SH2ZAM_MODEL_STATE = {
      ready, modelReady: !!valid, scene, detail, baseline, kernel, vertices: vertices || 0,
      referenceCycles: beforeCycles, kernelCycles: afterCycles, savedPercent, speedup,
      sourceCommit: valid ? run.sourceCommit : null
    };
    $('model-status').textContent = valid ? 'Recorded · SH-2 model' : 'No model capture';
    $('model-before-label').textContent = pathNames[baseline].toUpperCase();
    $('model-after-label').textContent = pathNames[kernel].toUpperCase();
    $('model-before-cycles').textContent = beforeCycles === null ? '—' : number(beforeCycles);
    $('model-after-cycles').textContent = afterCycles === null ? '—' : number(afterCycles);
    $('model-before-per-vertex').textContent = beforeCycles === null ? 'Not measured' : (beforeCycles / vertices).toFixed(1) + ' cycles / vertex';
    $('model-after-per-vertex').textContent = afterCycles === null ? 'Not measured' : (afterCycles / vertices).toFixed(1) + ' cycles / vertex';
    $('model-savings-label').textContent = ready && savedPercent < 0 ? 'EXTRA TRANSFORM COST' : 'TRANSFORM COST SAVED';
    $('model-savings').textContent = ready ? Math.abs(savedPercent).toFixed(1) + '%' : '—';
    $('model-savings').closest('article').classList.toggle('loss', ready && savedPercent < 0);
    $('model-speedup').textContent = !ready ? 'Awaiting model evidence'
      : speedup >= 1 ? speedup.toFixed(2) + '× speedup' : (1 / speedup).toFixed(2) + '× the baseline cost';
    const maxCycles = Math.max(beforeCycles || 0, afterCycles || 0);
    $('model-before-bar').style.width = ready ? (100 * beforeCycles / maxCycles) + '%' : '0%';
    $('model-after-bar').style.width = ready ? (100 * afterCycles / maxCycles) + '%' : '0%';
    $('model-workload').textContent = specs[scene][0] + ' · detail ' + detail + ' · '
      + (vertices ? number(vertices) : '—') + ' vertices · median of 21 poses';
    $('model-comparison-note').textContent = !valid
      ? 'No validated SH-2 capture is available for this workload.'
      : !ready ? 'This capture does not include an independent fixed-point baseline. Choose software float to inspect the recorded comparison.'
      : baseline === 'plain'
        ? 'Independent fixed-point C and SH2ZAM use the same inputs, rounding and output precision.'
        : 'The float reference runs as SH-2 software floating point. Both paths use the same quantized inputs.';
    $('model-proof').textContent = valid ? 'Fixed-point results: bit-exact' : 'No precision evidence loaded';
    $('model-float-error').textContent = valid
      ? 'Max float error: ' + (row.maximumWorldError * 65536).toFixed(3) + ' Q16.16 LSB' : '';
    $('model-provenance').textContent = valid
      ? 'SH-2 build ' + run.sourceCommit.slice(0, 8) + ' · Yabause ' + run.interpreterCommit.slice(0, 8)
        + ' · ELF ' + run.elfSHA256.slice(0, 12) + ' · ' + data.records.length + ' shared workloads'
      : '';
    $('model-evidence').href = valid ? run.evidenceUrl : 'https://github.com/lamb2k/SH2ZAM/tree/main/saturn/benchmarks/sh2-model';
    $('export').disabled = !valid && !mathData && !workloadData;
    for (const name of modelPaths) {
      const cycles = valid && positive(row.medianCycles?.[name]) ? row.medianCycles[name] : null;
      $('model-' + name).textContent = cycles === null ? 'Not measured' : number(cycles);
      $('model-' + name + '-per-vertex').textContent = cycles === null ? '—' : (cycles / vertices).toFixed(1);
      $('model-' + name + '-ratio').textContent = cycles === null || beforeCycles === null ? '—' : (beforeCycles / cycles).toFixed(2) + '×';
      const tr = document.querySelector('[data-model-row="' + name + '"]');
      tr.classList.toggle('selected', name === kernel);
      tr.classList.toggle('baseline-selected', name === baseline);
    }
    setCodeSnippets();
    publishVisualState();
  }

  function showHardwareStatus() {
    const data = window.SH2ZAM_SATURN_RESULTS, run = data?.run;
    const row = data?.records?.find(record => record.scene === scene && record.detail === detail);
    const captured = data?.schema === 3 && run?.environment === 'physical-console' && row;
    $('saturn-status').textContent = captured ? 'Console capture available' : 'Not measured';
    $('saturn-evidence').href = captured ? run.evidenceUrl : 'https://github.com/lamb2k/SH2ZAM/tree/main/saturn/benchmarks';
    $('saturn-evidence').textContent = captured ? 'Console evidence ↗' : 'Console benchmark ↗';
  }

  function validatedMath(data) {
    const run = data?.run, baselineCommit = '10786b64f4593d647d06d2a4a73e752c0c413dd0';
    if (data?.schema !== 1 || data.kernelSet !== 'previous-optimized-math-sh2-model-v1'
      || run?.environment !== 'isolated-sh2-interpreter' || run.hardwareMeasured !== false
      || run.browserTimingUsed !== false || run.hostResults !== false
      || !/^[a-f0-9]{40}$/.test(run.sourceCommit) || !/^[a-f0-9]{64}$/.test(run.elfSHA256)
      || data.baseline?.sourceCommit !== baselineCommit || data.comparison?.independentNonShazamBaseline !== false
      || data.batchSize !== 32 || data.samplesPerProfile !== 7 || data.mismatches !== 0
      || data.checkedOutputWords !== 114912 || !Array.isArray(data.records) || data.records.length !== 144) return null;
    const groups = new Map(), close = (a, b) => Number.isFinite(a) && Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b));
    let checkedOutputWords = 0;
    for (const row of data.records) {
      if (!row || !Object.hasOwn(mathFamilies, row.family) || !/^[a-z][a-z0-9_]{0,63}$/.test(row.operation)
        || !mathProfiles.includes(row.profile) || row.sampleCount !== data.samplesPerProfile
        || !Array.isArray(row.samples) || row.samples.length !== 7 || !row.samples.every(sample => sample?.mismatches === 0)
        || !positive(row.outputWords) || row.checkedOutputWords !== row.outputWords * data.batchSize * row.sampleCount
        || row.srSRetained !== true || row.srSOutputMatch !== true
        || !positive(row.medianCycles?.before) || !positive(row.medianCycles?.after)
        || !close(row.cyclesPerCall?.before, row.medianCycles.before / data.batchSize)
        || !close(row.cyclesPerCall?.after, row.medianCycles.after / data.batchSize)
        || !close(row.speedup, row.medianCycles.before / row.medianCycles.after)
        || !close(row.cyclesSavedPercent, (1 - row.medianCycles.after / row.medianCycles.before) * 100)) return null;
      const key = row.family + ':' + row.operation;
      if (!groups.has(key)) groups.set(key, new Set());
      const profiles = groups.get(key);
      if (profiles.has(row.profile)) return null;
      profiles.add(row.profile);
      checkedOutputWords += row.checkedOutputWords;
    }
    return checkedOutputWords === data.checkedOutputWords
      && [...groups.values()].every(profiles => profiles.size === mathProfiles.length) ? data : null;
  }

  function fillOptions(select, entries, selected) {
    select.replaceChildren(...entries.map(([value, label]) => {
      const option = document.createElement('option');
      option.value = value; option.textContent = label; return option;
    }));
    select.value = selected;
  }

  function selectMathOperations() {
    const operations = [...new Set(mathData.records.filter(row => row.family === mathFamily).map(row => row.operation))];
    if (!operations.includes(mathOperation)) mathOperation = operations[0];
    fillOptions($('math-operation'), operations.map(operation => [operation, 'sh2z_' + operation]), mathOperation);
  }

  function showMathResults() {
    const ready = !!mathData;
    const records = ready ? mathData.records.filter(row => row.family === mathFamily && row.operation === mathOperation) : [];
    window.SH2ZAM_MATH_STATE = {ready, family: mathFamily, operation: mathOperation,
      baselineCommit: ready ? mathData.baseline.sourceCommit : null,
      sourceCommit: ready ? mathData.run.sourceCommit : null, records};
    $('math-status').textContent = ready ? 'Recorded · SH-2 model' : 'Awaiting native capture';
    for (const profile of mathProfiles) {
      const row = records.find(record => record.profile === profile), prefix = 'math-' + profile;
      const cycles = value => value.toLocaleString('en-US', {maximumFractionDigits: 2});
      $(prefix + '-before').textContent = row ? cycles(row.cyclesPerCall.before) : '—';
      $(prefix + '-after').textContent = row ? cycles(row.cyclesPerCall.after) : '—';
      const cell = $(prefix + '-change');
      cell.classList.toggle('math-gain', !!row && row.cyclesSavedPercent > 0);
      cell.classList.toggle('math-loss', !!row && row.cyclesSavedPercent < 0);
      if (row) {
        const saved = row.cyclesSavedPercent;
        const percent = Math.abs(saved) < 0.05 && saved !== 0 ? '<0.1' : Math.abs(saved).toFixed(1);
        cell.textContent = saved === 0 ? 'No change' : percent + '% ' + (saved > 0 ? 'less' : 'more');
        const ratio = document.createElement('small');
        ratio.textContent = saved === 0 ? 'Same cost' : saved > 0
          ? row.speedup.toFixed(2) + '× speedup' : (1 / row.speedup).toFixed(2) + '× prior cost';
        cell.append(ratio);
      } else cell.textContent = 'Not measured';
      $(prefix + '-precision').textContent = row ? 'Bit-exact' : 'Awaiting checks';
      $(prefix + '-precision').classList.toggle('math-exact', !!row);
    }
    $('math-proof').classList.toggle('pending', !ready);
    $('math-proof').textContent = ready
      ? 'sh2z_' + mathOperation + ' · ' + number(records.reduce((sum, row) => sum + row.checkedOutputWords, 0))
        + ' output words checked · bit-exact across all three profiles.'
      : window.SH2ZAM_MATH_MODEL ? 'The loaded math data did not pass its SH-2 evidence checks. No performance figures are shown.'
        : 'Awaiting a validated native SH-2 capture. No math performance figures are available yet.';
    $('math-sampling').textContent = ready
      ? mathData.batchSize + '-call batches · median of ' + mathData.samplesPerProfile
        + ' fixture sets per profile · full-range inputs can take safe fallbacks. Instruction-model costs, not console timings.' : '';
    $('math-provenance').textContent = ready
      ? new Set(mathData.records.map(row => row.operation)).size + ' operations · prior 10786b6 → '
        + mathData.run.sourceCommit.slice(0, 8) + ' · ELF ' + mathData.run.elfSHA256.slice(0, 12) : '';
    const source = 'https://github.com/lamb2k/SH2ZAM/tree/main/saturn/benchmarks/math-model';
    let evidence = source;
    if (ready) {
      try {
        const url = new URL(mathData.run.evidenceUrl);
        if (url.origin === 'https://github.com' && url.pathname.startsWith('/lamb2k/SH2ZAM/')) evidence = url.href;
      } catch (_) { /* A malformed link cannot override the fixed source URL. */ }
    }
    $('math-evidence').href = evidence;
    $('export').disabled = !mathData && !modelData && !workloadData;
  }

  function loadMathResults() {
    mathData = validatedMath(window.SH2ZAM_MATH_MODEL);
    if (mathData) {
      const families = Object.entries(mathFamilies).filter(([family]) => mathData.records.some(row => row.family === family));
      if (!families.some(([family]) => family === mathFamily)) mathFamily = families[0][0];
      fillOptions($('math-family'), families, mathFamily);
      selectMathOperations();
      $('math-family').disabled = false; $('math-operation').disabled = false;
    }
    showMathResults();
  }

  function validatedWorkloads(data) {
    const run = data?.run, prior = '3374d77c4625749cc932cafa4385ad18db811e9b';
    if (data?.schema !== 1 || data.kernelSet !== 'exact-workloads-sh2-model-v1'
      || data.baseline?.sourceCommit !== prior || data.comparison?.priorLibraryCommit !== prior
      || data.comparison?.claimsUniversalTenfoldGain !== false || data.samplesPerCase !== 7
      || data.mismatches !== 0 || !Array.isArray(data.records) || data.records.length !== 44
      || run?.environment !== 'isolated-sh2-interpreter' || run.hardwareMeasured !== false
      || run.browserTimingUsed !== false || run.hostResults !== false || run.setupIncluded !== true
      || run.wholeBatch !== true || run.cpuCount !== 1 || run.memoryModel !== 'flat RAM'
      || run.toolchainVerifiedInELF !== true || run.interpreterModified !== false
      || run.interpreterCommit !== '7e38821dbac265490f115e163c523a939acda759'
      || !/^[a-f0-9]{40}$/.test(run.sourceCommit) || !/^[a-f0-9]{64}$/.test(run.elfSHA256)
      || !/^[a-f0-9]{64}$/.test(run.rawCaptureSHA256)) return null;
    const seen = new Set(), ids = new Set(), close = (a, b) => Number.isFinite(a) && Math.abs(a - b) <= 1e-10 * Math.max(1, Math.abs(b));
    const median = values => [...values].sort((a, b) => a - b)[3];
    let words = 0, comparisons = 0;
    for (const row of data.records) {
      const spec = workloadSpecs[row?.group], key = row?.group + ':' + row?.profile + ':' + row?.size;
      if (!spec || !spec.sizes.includes(row.size) || !spec.profiles.includes(row.profile)
        || seen.has(key) || typeof row.id !== 'string' || ids.has(row.id)
        || JSON.stringify(row.paths) !== JSON.stringify(spec.paths)
        || row.queries !== (row.group === 'spatial' ? 128 : 0)
        || row.sampleCount !== 7 || row.setupIncluded !== true
        || !Array.isArray(row.samples) || row.samples.length !== 7
        || row.guards?.srSRetained !== true || row.guards?.srSOutputMatch !== true || row.guards?.noMacWithS !== true) return null;
      const outputWords = row.group === 'spatial' ? 128 * (Math.ceil(row.size / 32) + 1) : row.size * (['rotation', 'normalize'].includes(row.group) ? 3 : 1);
      if (row.outputWords !== outputWords || row.checkedOutputWords !== 7 * outputWords
        || row.checkedComparisons !== 7 * outputWords * (row.paths.length - 1)) return null;
      for (const sample of row.samples) {
        if (sample?.mismatches !== 0 || !/^[a-f0-9]{8}$/.test(sample.inputChecksum)) return null;
        const checksums = new Set();
        for (const name of spec.paths) {
          const cost = sample.paths?.[name];
          if (!positive(cost?.cycles) || !positive(cost?.instructions) || !/^[a-f0-9]{8}$/.test(cost?.checksum)) return null;
          for (const metric of ['dataReads', 'dataWrites', 'macInstructions', 'dmulsInstructions'])
            if (!Number.isSafeInteger(cost[metric]) || cost[metric] < 0) return null;
          if (cost.macInstructions + cost.dmulsInstructions > cost.instructions) return null;
          checksums.add(cost.checksum);
        }
        if (checksums.size !== 1) return null;
      }
      const divisor = row.group === 'spatial' ? row.queries : row.size;
      for (const name of spec.paths) {
        const cycles = median(row.samples.map(sample => sample.paths[name].cycles));
        if (row.medianCycles?.[name] !== cycles || !close(row.cyclesPerItem?.[name], cycles / divisor)) return null;
        for (const control of spec.baselines) {
          const ratio = row.relativeTo?.[control]?.[name];
          if (!close(ratio?.speedup, row.medianCycles[control] / cycles)
            || !close(ratio?.cyclesSavedPercent, 100 * (1 - cycles / row.medianCycles[control]))) return null;
        }
      }
      seen.add(key); ids.add(row.id); words += row.checkedOutputWords; comparisons += row.checkedComparisons;
    }
    return words === data.checkedOutputWords && comparisons === data.checkedComparisons ? data : null;
  }

  function strongestWorkloadControl(row) {
    return workloadSpecs[row.group].baselines.reduce((best, name) => row.medianCycles[name] < row.medianCycles[best] ? name : best);
  }
  function workloadRatio(beforeCycles, afterCycles) {
    return beforeCycles === afterCycles ? 'Same cost' : beforeCycles > afterCycles
      ? (beforeCycles / afterCycles).toFixed(2) + '× speedup'
      : (afterCycles / beforeCycles).toFixed(2) + '× control cost';
  }
  function workloadChange(beforeCycles, afterCycles) {
    if (beforeCycles === afterCycles) return 'No change';
    const percent = Math.abs(100 * (1 - afterCycles / beforeCycles));
    return (percent < .05 ? '<0.1' : percent.toFixed(1)) + '% ' + (afterCycles < beforeCycles ? 'less' : 'more');
  }
  function workloadCell(tag, text, className) {
    const cell = document.createElement(tag); cell.textContent = text;
    if (className) cell.className = className;
    if (tag === 'th') cell.scope = 'row';
    return cell;
  }
  function selectWorkload(group, profile, size) {
    if (!workloadData || !Object.hasOwn(workloadSpecs, group)) return;
    workloadGroup = group;
    const spec = workloadSpecs[group];
    workloadProfile = spec.profiles.includes(profile) ? profile : spec.profiles[0];
    workloadSize = spec.sizes.includes(size) ? size : spec.sizes[spec.sizes.length - 1];
    workloadBaseline = ['spatial', 'rotation'].includes(group) ? 'strongest' : 'prior';
    workloadKernel = group === 'rotation' ? 'mac' : spec.kernels[0];
    fillOptions($('workload-group'), Object.entries(workloadSpecs).map(([key, value]) => [key, value.label]), group);
    fillOptions($('workload-profile'), spec.profiles.map(name => [name, workloadProfileNames[name]]), workloadProfile);
    fillOptions($('workload-size'), spec.sizes.map(value => [String(value), number(value) + (group === 'spatial' ? ' boxes' : group === 'rotation' ? ' vectors' : ' calls')]), String(workloadSize));
    const controls = spec.baselines.map(name => [name, workloadPathNames[name]]);
    if (controls.length > 1) controls.unshift(['strongest', 'Fastest recorded control']);
    fillOptions($('workload-baseline'), controls, workloadBaseline);
    fillOptions($('workload-kernel'), spec.kernels.map(name => [name, workloadPathNames[name]]), workloadKernel);
    for (const name of ['group', 'profile', 'size', 'baseline', 'kernel']) $('workload-' + name).disabled = false;
    showWorkloadResults();
  }

  function showWorkloadHighlights() {
    for (const group of ['rotation', 'spatial']) {
      const row = workloadData.records.find(value => value.group === group && value.size === 512 && value.profile === (group === 'rotation' ? 'bounded' : 'sparse'));
      const control = group === 'rotation' ? strongestWorkloadControl(row)
        : row.medianCycles.inline_brute <= row.medianCycles.inline_scatter ? 'inline_brute' : 'inline_scatter';
      const path = group === 'rotation' ? 'mac' : 'index';
      const beforeCycles = row.medianCycles[control], afterCycles = row.medianCycles[path];
      const headline = $('workload-' + group + '-highlight');
      headline.textContent = workloadRatio(beforeCycles, afterCycles);
      headline.classList.toggle('math-loss', afterCycles > beforeCycles);
      if (group === 'rotation') {
        $('workload-rotation-control').textContent = 'Against ' + workloadPathNames[control].toLowerCase();
        $('workload-rotation-other').textContent = ['scalar', 'composed', 'composed_mac'].map(name =>
          (row.medianCycles[name] / afterCycles).toFixed(2) + '× vs ' + ({scalar: 'repeated calls', composed: 'matrix + C', composed_mac: 'matrix + MAC.L'}[name])).join(' · ');
      } else {
        $('workload-spatial-control').textContent = 'Against fastest independent C · '
          + (control === 'inline_brute' ? 'packed output' : 'sparse hit writes');
        $('workload-spatial-other').textContent = ['inline_brute', 'inline_scatter', 'api_brute'].map(name =>
          (row.medianCycles[name] / afterCycles).toFixed(2) + '× vs '
            + ({inline_brute: 'packed output', inline_scatter: 'sparse writes', api_brute: 'API loop'}[name])).join(' · ')
          + ' · build included.';
      }
    }
    let faster = 0, same = 0, slower = 0, tenfold = 0;
    const rows = workloadData.records.map(row => {
      const control = strongestWorkloadControl(row), kernel = row.group === 'rotation' ? 'mac' : row.group === 'spatial' ? 'index' : 'current';
      const beforeCycles = row.medianCycles[control], afterCycles = row.medianCycles[kernel];
      if (afterCycles < beforeCycles) faster++; else if (afterCycles > beforeCycles) slower++; else same++;
      if (beforeCycles / afterCycles >= 10) tenfold++;
      const tr = document.createElement('tr'), name = workloadCell('td', '');
      const button = document.createElement('button'); button.type = 'button'; button.className = 'workload-case-select';
      button.textContent = workloadSpecs[row.group].label;
      button.addEventListener('click', () => {
        selectWorkload(row.group, row.profile, row.size);
        $('workload-group').focus();
      });
      name.append(button);
      tr.append(name, workloadCell('td', workloadProfileNames[row.profile] + ' / ' + number(row.size)),
        workloadCell('td', workloadPathNames[control]), workloadCell('td', number(afterCycles)),
        workloadCell('td', workloadChange(beforeCycles, afterCycles) + ' · ' + workloadRatio(beforeCycles, afterCycles), afterCycles < beforeCycles ? 'math-gain' : afterCycles > beforeCycles ? 'math-loss' : ''));
      return tr;
    });
    $('workload-all-cases').replaceChildren(...rows);
    $('workload-coverage').textContent = 'All 44 cases against their fastest recorded controls: '
      + faster + ' faster · ' + same + ' unchanged · ' + slower + ' slower. '
      + tenfold + ' of 44 reach 10×. Every case is retained below.';
    document.querySelectorAll('[data-workload-example]').forEach(button => { button.disabled = false; });
  }

  function showWorkloadResults() {
    const row = workloadData?.records.find(value => value.group === workloadGroup && value.profile === workloadProfile && value.size === workloadSize);
    if (!row) {
      const rejected = !!window.SH2ZAM_WORKLOAD_MODEL;
      window.SH2ZAM_WORKLOAD_STATE = {ready: false, rejected};
      $('workload-status').textContent = rejected ? 'Evidence rejected' : 'Awaiting native capture';
      $('workload-proof').textContent = rejected
        ? 'The loaded workload data failed its native evidence checks. No performance figures are shown.'
        : 'Awaiting validated native SH-2 evidence. No workload gains are claimed yet.';
      $('workload-proof').classList.add('pending');
      return;
    }
    const control = workloadBaseline === 'strongest' ? strongestWorkloadControl(row) : workloadBaseline;
    const beforeCycles = row.medianCycles[control], afterCycles = row.medianCycles[workloadKernel];
    const divisor = row.group === 'spatial' ? row.queries : row.size, unit = row.group === 'spatial' ? 'query' : row.group === 'rotation' ? 'vector' : 'call';
    const savings = 100 * (1 - afterCycles / beforeCycles), perItem = cycles => (cycles / divisor).toLocaleString('en-US', {maximumFractionDigits: 2}) + ' cycles / ' + unit;
    window.SH2ZAM_WORKLOAD_STATE = {ready: true, group: workloadGroup, profile: workloadProfile, size: workloadSize,
      baselineChoice: workloadBaseline, baseline: control, kernel: workloadKernel, beforeCycles, afterCycles,
      speedup: beforeCycles / afterCycles, savedPercent: savings, sourceCommit: workloadData.run.sourceCommit, record: row};
    $('workload-status').textContent = 'Recorded · SH-2 model';
    $('workload-before-label').textContent = workloadPathNames[control].toUpperCase();
    $('workload-after-label').textContent = workloadPathNames[workloadKernel].toUpperCase();
    $('workload-before-cycles').textContent = number(beforeCycles); $('workload-after-cycles').textContent = number(afterCycles);
    $('workload-before-unit').textContent = perItem(beforeCycles); $('workload-after-unit').textContent = perItem(afterCycles);
    $('workload-savings-label').textContent = savings < 0 ? 'EXTRA WORKLOAD COST' : 'WORKLOAD COST SAVED';
    $('workload-savings').textContent = (Math.abs(savings) > 0 && Math.abs(savings) < .05 ? '<0.1' : Math.abs(savings).toFixed(1)) + '%';
    $('workload-savings').closest('article').classList.toggle('loss', savings < 0);
    $('workload-speedup').textContent = workloadRatio(beforeCycles, afterCycles);
    $('workload-context').textContent = workloadSpecs[row.group].label + ' · ' + workloadProfileNames[row.profile]
      + ' · ' + number(row.size) + (row.group === 'spatial' ? ' boxes / 128 queries' : row.group === 'rotation' ? ' vectors' : ' calls')
      + ' · median of seven changing-input samples.';
    $('workload-baseline-note').textContent = row.group === 'rotation'
      ? (control === 'scalar' ? 'Repeated-call control selected. ' : workloadBaseline === 'strongest' ? 'The fastest existing control is selected for this case. ' : 'Existing composed control selected. ')
        + 'All rotation controls already use SH2ZAM. Both composed paths remain visible so shared-setup gains can be assessed separately from improvements beyond composition.'
      : row.group === 'spatial' ? (workloadBaseline === 'strongest' ? 'The fastest recorded control is selected for this case. ' : 'Selected spatial control shown above. ')
        + 'Both independent C strategies prepare each query once and use the same exact predicate: packed output writes one word at a time; sparse hit writes update the bitset on a match. Both strategies and the prior API loop remain visible.'
        : 'Latest optimized revision 3374d77 → this capture. The 48-operation table below keeps the earlier 10786b6 comparison separately labeled.';
    $('workload-unit-heading').textContent = 'Cycles / ' + unit;
    $('workload-table-caption').textContent = 'Whole-workload medians · setup included · all ' + row.paths.length + ' paths';
    $('workload-paths').replaceChildren(...row.paths.map(name => {
      const cycles = row.medianCycles[name], tr = document.createElement('tr'), label = workloadCell('th', workloadPathNames[name]);
      if (name === control) label.append(workloadCell('small', 'Selected control'));
      else if (name === strongestWorkloadControl(row)) label.append(workloadCell('small', 'Fastest existing control'));
      tr.classList.toggle('selected', name === workloadKernel); tr.classList.toggle('baseline-selected', name === control);
      tr.append(label, workloadCell('td', number(cycles)), workloadCell('td', (cycles / divisor).toLocaleString('en-US', {maximumFractionDigits: 2})),
        workloadCell('td', (beforeCycles / cycles).toFixed(3) + '×', cycles > beforeCycles ? 'math-loss' : cycles < beforeCycles ? 'math-gain' : ''));
      return tr;
    }));
    $('workload-proof').classList.remove('pending');
    $('workload-proof').textContent = 'Bit-exact: ' + number(row.checkedOutputWords) + ' output words · '
      + number(row.checkedComparisons) + ' path comparisons · zero mismatches. SR.S preserved; native MAC.L disabled with SR.S=1.';
    $('workload-scope').textContent = row.group === 'rotation'
      ? 'One shared quaternion setup per composed or batch call. When MAC.L is attempted, the input scan and range qualification are included. New batches below eight vectors skip MAC preparation; one vector uses the scalar call. Full-range and SR.S fallbacks stay in the results.'
      : row.group === 'spatial' ? 'One index build + all 128 queries are measured together. Results retain dense, crowded, and full-range inputs. Rebuild when boxes change.'
        : 'All 32 API calls, input loads, and result writes are included. No relaxed precision or approximate API is substituted.';
    const header = document.createElement('tr');
    for (const label of ['Sample / input', ...row.paths.map(name => workloadPathNames[name]), 'Output checksum']) {
      const cell = workloadCell('th', label); cell.scope = 'col'; header.append(cell);
    }
    $('workload-samples-head').replaceChildren(header);
    $('workload-samples').replaceChildren(...row.samples.map((sample, i) => {
      const tr = document.createElement('tr');
      tr.append(workloadCell('th', String(i + 1) + ' / ' + sample.inputChecksum),
        ...row.paths.map(name => workloadCell('td', number(sample.paths[name].cycles))),
        workloadCell('td', sample.paths[row.paths[0]].checksum));
      return tr;
    }));
    $('workload-samples-note').textContent = 'Modeled cycles for the selected case. Identical outputs are checked word by word across all paths. The export includes every sample’s instruction, memory-access, and multiplier counters for all 44 cases.';
    const macs = row.samples.reduce((sum, sample) => sum + sample.paths[workloadKernel].macInstructions, 0);
    $('workload-dispatch').textContent = 'Selected path: ' + number(macs) + ' MAC.L instructions across these seven samples. A zero count includes paths using exact fallbacks.';
    const run = workloadData.run;
    $('workload-provenance').textContent = '44 complete cases · ' + number(workloadData.checkedComparisons)
      + ' exact comparisons · source ' + run.sourceCommit.slice(0, 8) + ' · prior 3374d77 · compiler ' + run.compiler;
    $('workload-flags').textContent = run.compileFlags + '\nELF SHA-256: ' + run.elfSHA256
      + '\nRaw capture SHA-256: ' + run.rawCaptureSHA256 + '\nYabause: ' + run.interpreterCommit;
    try {
      const url = new URL(run.evidenceUrl);
      if (url.origin === 'https://github.com' && url.pathname.startsWith('/lamb2k/SH2ZAM/actions/')) $('workload-evidence').href = url.href;
    } catch (_) { /* Retain the fixed source link if provenance is malformed. */ }
    $('workload-export').disabled = false; $('export').disabled = false;
  }

  function loadWorkloadResults() {
    workloadData = validatedWorkloads(window.SH2ZAM_WORKLOAD_MODEL);
    if (workloadData) { showWorkloadHighlights(); selectWorkload('rotation', 'bounded', 512); }
    else showWorkloadResults();
  }

  function loadScene() {
    mesh = makeMesh ? makeMesh(scene, detail) : null;
    const count = mesh ? mesh.vertices.length / 3 : 0;
    $('scene-title').textContent = specs[scene][0];
    $('scene-caption').textContent = specs[scene][1];
    $('mesh-count').textContent = mesh ? number(count) + ' vertices / ' + number(mesh.faces.length) + ' triangles' : 'Visual preview unavailable';
    showModelResults();
    showHardwareStatus();
    if (!wasm || !mesh) return;
    if (count > wasm.capacity()) throw new Error('Mesh exceeds the WebAssembly buffer.');
    input = new Int32Array(wasm.memory.buffer, wasm.input_ptr(), count * 3);
    input.set(mesh.vertices.map(value => Math.round(value * 65536)));
    if (!wasm.prepare(count)) throw new Error('The C bridge rejected the mesh.');
    before = new Float32Array(wasm.memory.buffer, wasm.before_ptr(), count * 3);
    beforeBits = new Uint32Array(wasm.memory.buffer, wasm.before_ptr(), count * 3);
    general = new Int32Array(wasm.memory.buffer, wasm.general_ptr(), count * 3);
    after = new Int32Array(wasm.memory.buffer, wasm.after_ptr(), count * 3);
    draw();
  }

  function resize() {
    const box = $('stage').getBoundingClientRect(); width = box.width; height = box.height;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    canvases.forEach((canvas, i) => {
      canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
      if (contexts[i]) contexts[i].setTransform(dpr, 0, 0, dpr, 0, 0);
    });
    if (wasm) draw();
  }
  function project(x, y, z) {
    const scale = Math.min(height, width) * (scene === 'cubes' ? 0.95 : 1.25) / (8 - z);
    return [width / 2 + x * scale, height / 2 - y * scale];
  }
  function render(ctx, points, divisor) {
    ctx.clearRect(0, 0, width, height);
    const gradient = ctx.createRadialGradient(width * .5, height * .46, 0, width * .5, height * .46, width * .66);
    gradient.addColorStop(0, '#12313d'); gradient.addColorStop(1, '#080d15');
    ctx.fillStyle = gradient; ctx.fillRect(0, 0, width, height);
    for (let i = 0; i < 90; i++) {
      const x = ((i * 7919) % 1009) / 1009 * width, y = ((i * 3571) % 1013) / 1013 * height;
      ctx.fillStyle = i % 4 ? '#63869777' : '#ace9e6aa'; ctx.fillRect(x, y, i % 6 ? 1 : 2, 1);
    }
    const pts = [];
    for (let i = 0; i < points.length; i += 3) {
      const x = points[i] / divisor, y = points[i + 1] / divisor, z = points[i + 2] / divisor;
      pts.push([x, y, z, ...project(x, y, z)]);
    }
    const sorted = mesh.faces.map(face => ({face, z: face.indices.reduce((sum, i) => sum + pts[i][2], 0)})).sort((a, b) => a.z - b.z);
    const palette = [[181, 73, 49], [192, 48, 56], [163, 60, 66]];
    for (const {face} of sorted) {
      const [a, b, c] = face.indices.map(i => pts[i]);
      const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]], length = Math.hypot(...n);
      if (length < 1e-10) continue;
      const light = 0.32 + 0.68 * Math.abs((n[0] * -.35 + n[1] * .55 + n[2] * .76) / length);
      const [h, s, l] = palette[face.color];
      ctx.fillStyle = 'hsl(' + h + ' ' + s + '% ' + l * light + '%)';
      ctx.beginPath(); ctx.moveTo(a[3], a[4]); ctx.lineTo(b[3], b[4]); ctx.lineTo(c[3], c[4]); ctx.closePath(); ctx.fill();
      ctx.strokeStyle = '#071b2455'; ctx.lineWidth = .45; ctx.stroke();
    }
    return pts;
  }
  function checkPrecision() {
    maxWorldError = 0; maxGeneralWorldError = 0; fixedMismatches = 0;
    let refChecksum = 0, generalChecksum = 0, optimizedChecksum = 0;
    for (let i = 0; i < after.length; i++) {
      maxWorldError = Math.max(maxWorldError, Math.abs(before[i] - after[i] / 65536));
      maxGeneralWorldError = Math.max(maxGeneralWorldError, Math.abs(before[i] - general[i] / 65536));
      if (general[i] !== after[i]) fixedMismatches++;
      refChecksum = (Math.imul(refChecksum, 33) + beforeBits[i]) >>> 0;
      generalChecksum = (Math.imul(generalChecksum, 33) + (general[i] >>> 0)) >>> 0;
      optimizedChecksum = (Math.imul(optimizedChecksum, 33) + (after[i] >>> 0)) >>> 0;
    }
    const hex = value => value.toString(16).padStart(8, '0');
    outputChecksums = {reference: hex(refChecksum), general: hex(generalChecksum), optimized: hex(optimizedChecksum)};
    outputsValid = fixedMismatches === 0 && [maxError, maxWorldError, maxGeneralWorldError].every(Number.isFinite)
      && maxWorldError <= MAX_WORLD_ERROR && maxGeneralWorldError <= MAX_WORLD_ERROR;
    $('error').textContent = Number.isFinite(maxError) ? maxError.toFixed(3) + ' px' : 'Invalid';
    $('world-error').textContent = Number.isFinite(maxWorldError) ? maxWorldError.toFixed(7) : 'Invalid';
    $('fixed-equality').textContent = outputsValid ? 'Fixed outputs: bit-exact'
      : fixedMismatches ? fixedMismatches + ' differing components' : 'Visual precision check failed';
    $('fixed-equality').classList.toggle('mismatch', !outputsValid);
  }
  function draw() {
    if (!wasm || !mesh || !input || !contexts.every(Boolean)) return;
    const phase = seconds / 12, ax = Math.round(5200 + 3000 * Math.sin(phase * TAU)) & 65535, ay = Math.floor(phase * 65536) & 65535;
    const x = ax * TAU / 65536, y = ay * TAU / 65536;
    wasm.setup(ax, ay, Math.sin(x), Math.cos(x), Math.sin(y), Math.cos(y));
    wasm.before(); wasm.general(); wasm.after();
    const a = render(contexts[0], before, 1), b = render(contexts[1], after, 65536);
    maxError = 0;
    for (let i = 0; i < a.length; i++) maxError = Math.max(maxError, Math.hypot(a[i][3] - b[i][3], a[i][4] - b[i][4]));
    checkPrecision();
    $('play').textContent = playing ? 'Pause' : 'Play';
    $('play-state').textContent = playing ? '12 SECOND LOOP' : 'PAUSED';
    publishVisualState();
  }
  function frame(now) {
    if (last && playing && !document.hidden) {
      seconds = (seconds + Math.min((now - last) / 1000, 0.1)) % 12;
      draw();
    }
    last = now;
    requestAnimationFrame(frame);
  }
  function exportModel() {
    if ((!modelData || !modelRow) && !mathData && !workloadData) return;
    const state = window.SH2ZAM_MODEL_STATE;
    const report = {
      schema: 'sh2zam-model-selection-v1', environment: 'isolated-sh2-interpreter',
      ...(modelData && modelRow ? {
      selection: {scene, detail, baseline, kernel},
      comparison: {
        referenceCycles: state.referenceCycles, kernelCycles: state.kernelCycles,
        savedPercent: state.savedPercent, speedup: state.speedup
      },
      evidence: {
        schema: modelData.schema, kernelSet: modelData.kernelSet, fixtureSHA256: modelData.fixtureSHA256,
        run: modelData.run, guards: modelData.guards, records: [modelRow]
      }
      } : {}),
      ...(mathData ? {mathLibrary: {selection: {family: mathFamily, operation: mathOperation}, evidence: mathData}} : {}),
      ...(workloadData ? {workloads: {selection: {group: workloadGroup, profile: workloadProfile, size: workloadSize,
        baseline: workloadBaseline, kernel: workloadKernel}, evidence: workloadData}} : {})
    };
    const url = URL.createObjectURL(new Blob([JSON.stringify(report, null, 2)], {type: 'application/json'}));
    const link = document.createElement('a');
    link.href = url; link.download = 'sh2zam-sh2-model-' + scene + '-d' + detail + '.json';
    link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  function selectButtons(attribute, value) {
    document.querySelectorAll('[' + attribute + ']').forEach(button => {
      const active = button.getAttribute(attribute) === value;
      button.classList.toggle('active', active); button.setAttribute('aria-pressed', String(active));
    });
  }
  $('play').addEventListener('click', () => { playing = !playing; last = 0; draw(); });
  $('detail').addEventListener('input', event => {
    detail = Number(event.target.value); $('detail-value').textContent = detail; loadScene();
  });
  document.querySelectorAll('[data-scene]').forEach(button => button.addEventListener('click', () => {
    scene = button.dataset.scene; selectButtons('data-scene', scene); loadScene();
  }));
  document.querySelectorAll('[data-baseline]').forEach(button => button.addEventListener('click', () => {
    baseline = button.dataset.baseline; selectButtons('data-baseline', baseline); showModelResults();
  }));
  document.querySelectorAll('[data-kernel]').forEach(button => button.addEventListener('click', () => {
    kernel = button.dataset.kernel; selectButtons('data-kernel', kernel); showModelResults();
  }));
  $('export').addEventListener('click', exportModel);
  $('math-family').addEventListener('change', event => {
    if (!mathData) return;
    mathFamily = event.target.value; selectMathOperations(); showMathResults();
  });
  $('math-operation').addEventListener('change', event => {
    if (!mathData) return;
    mathOperation = event.target.value; showMathResults();
  });
  $('workload-group').addEventListener('change', event => selectWorkload(event.target.value));
  $('workload-profile').addEventListener('change', event => {
    if (!workloadData || !workloadSpecs[workloadGroup].profiles.includes(event.target.value)) return;
    workloadProfile = event.target.value; showWorkloadResults();
  });
  $('workload-size').addEventListener('change', event => {
    const size = Number(event.target.value);
    if (!workloadData || !workloadSpecs[workloadGroup].sizes.includes(size)) return;
    workloadSize = size; showWorkloadResults();
  });
  $('workload-baseline').addEventListener('change', event => {
    const value = event.target.value;
    if (!workloadData || (value !== 'strongest' && !workloadSpecs[workloadGroup].baselines.includes(value))) return;
    workloadBaseline = value; showWorkloadResults();
  });
  $('workload-kernel').addEventListener('change', event => {
    if (!workloadData || !workloadSpecs[workloadGroup].kernels.includes(event.target.value)) return;
    workloadKernel = event.target.value; showWorkloadResults();
  });
  document.querySelectorAll('[data-workload-example]').forEach(button => button.addEventListener('click', () => {
    const group = button.dataset.workloadExample;
    selectWorkload(group, group === 'spatial' ? 'sparse' : 'bounded', 512);
  }));
  $('workload-export').addEventListener('click', () => {
    if (!workloadData) return;
    const url = URL.createObjectURL(new Blob([JSON.stringify(workloadData, null, 2)], {type: 'application/json'}));
    const link = document.createElement('a'); link.href = url; link.download = 'sh2zam-workload-evidence-' + workloadData.run.sourceCommit.slice(0, 8) + '.json';
    link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  reduced.addEventListener('change', event => { if (event.matches) { playing = false; draw(); } });
  new ResizeObserver(resize).observe($('stage'));

  async function initVisual() {
    try {
      if (!makeMesh) throw new Error('The mesh module is unavailable.');
      if (!contexts.every(Boolean)) throw new Error('This browser cannot open the visual canvas.');
      if (!window.SH2ZAM_WASM_BASE64) throw new Error('The visual WebAssembly module is unavailable.');
      const bytes = Uint8Array.from(atob(window.SH2ZAM_WASM_BASE64), character => character.charCodeAt(0));
      const module = await WebAssembly.instantiate(bytes, {}); wasm = module.instance.exports;
      $('backend').textContent = 'ACTUAL SH2ZAM C · VISUAL PRECISION CHECK';
      $('play').disabled = false;
      loadScene(); resize(); requestAnimationFrame(frame);
    } catch (error) {
      visualError = error.message;
      $('backend').textContent = 'VISUAL PREVIEW UNAVAILABLE';
      $('failure').hidden = false;
      $('failure').textContent = 'Visual preview unavailable. ' + error.message + ' The recorded SH-2 results remain available.';
      $('fixed-equality').textContent = 'Visual check unavailable';
      $('play-state').textContent = 'PREVIEW UNAVAILABLE';
      publishVisualState();
    }
  }
  // Model evidence does not depend on WebAssembly or animation starting.
  loadWorkloadResults();
  loadMathResults();
  loadScene();
  initVisual();
})();
