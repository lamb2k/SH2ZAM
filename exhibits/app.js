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
    $('export').disabled = !valid;
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
    if (!modelData || !modelRow) return;
    const state = window.SH2ZAM_MODEL_STATE;
    const report = {
      schema: 'sh2zam-model-selection-v1', environment: 'isolated-sh2-interpreter',
      selection: {scene, detail, baseline, kernel},
      comparison: {
        referenceCycles: state.referenceCycles, kernelCycles: state.kernelCycles,
        savedPercent: state.savedPercent, speedup: state.speedup
      },
      evidence: {
        schema: modelData.schema, kernelSet: modelData.kernelSet, fixtureSHA256: modelData.fixtureSHA256,
        run: modelData.run, guards: modelData.guards, records: [modelRow]
      }
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
  loadScene();
  initVisual();
})();
