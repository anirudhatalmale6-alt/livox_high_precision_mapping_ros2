// Mapper dashboard front-end. Talks to the stdlib server: live status over SSE,
// actions over POST. No framework, no build step - just works on the Pi.
(function () {
  'use strict';
  var $ = function (id) { return document.getElementById(id); };

  function pill(el, text, cls) {
    el.textContent = text;
    el.className = 'pill' + (cls ? ' ' + cls : '');
  }

  function toast(msg, bad) {
    var t = $('toast');
    t.textContent = msg;
    t.className = 'toast show' + (bad ? ' bad' : '');
    setTimeout(function () { t.className = 'toast'; }, 2600);
  }

  function post(path, body) {
    return fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    }).then(function (r) { return r.json(); })
      .then(function (j) { toast(j.message || (j.ok ? 'OK' : 'Failed'), !j.ok); return j; })
      .catch(function (e) { toast('Request failed: ' + e, true); });
  }

  function confirmPost(path, question, body) {
    if (window.confirm(question)) { post(path, body); }
  }

  // ---- render a status snapshot ----------------------------------------
  function render(s) {
    // connection
    $('connDot').className = 'dot' + (s.connected ? ' on' : '');
    $('connText').textContent = s.connected ? 'Connected' : 'No LiDAR';

    // logging lifecycle
    var stateLabel = {
      idle: 'Idle', initial_data_logging: 'Initial data logging',
      active_logging: 'Active logging', stopping: 'Stopping'
    }[s.logging_state] || s.logging_state;
    $('logState').textContent = stateLabel;
    var extra = s.log_message || '';
    if (s.logging_state === 'active_logging' && s.elapsed_s) {
      extra = 'Recording ' + fmtTime(s.elapsed_s) + (s.usb && s.usb.path ? ' -> ' + s.usb.path : '');
    } else if (s.logging_state === 'idle' && s.last_map) {
      extra = 'last map: ' + s.last_map;
    }
    if (s.logging_state === 'idle' && !(s.usb && s.usb.mounted && s.usb.free_gb >= 1)) {
      extra = 'insert a USB drive to enable logging';
    }
    $('logMsg').textContent = extra ? ' - ' + extra : '';
    var logging = (s.logging_state === 'active_logging' || s.logging_state === 'initial_data_logging');
    $('recDot').className = 'recdot' + (logging ? ' live' : '');

    // buttons: block start unless idle + USB healthy; block stop unless logging
    var usbOk = s.usb && s.usb.mounted && s.usb.free_gb >= 1;
    $('btnStart').disabled = logging || !usbOk;
    $('btnStop').disabled = !logging;

    // sensor health
    pill($('stLidar'), s.lidar.ok ? 'Streaming ' + s.lidar.rate_hz + ' Hz' : 'Waiting', s.lidar.ok ? 'ok' : 'bad');
    pill($('stImu'), s.imu.ok ? 'Streaming ' + s.imu.rate_hz + ' Hz' : 'Waiting', s.imu.ok ? 'ok' : 'bad');
    var gfix = s.gnss.fix + (s.gnss.sats ? ' - ' + s.gnss.sats + ' sats' : '');
    var gcls = s.gnss.ok ? (/rtk/i.test(s.gnss.fix) ? 'ok' : 'warn') : 'bad';
    pill($('stGnss'), s.gnss.ok ? gfix : 'No fix', gcls);
    // Which clock the scan is stamped with. Red when it is the computer clock:
    // that is a silent fallback, and a scan on a wrong clock looks normal until
    // the data is analysed.
    var ts = s.time_sync || {ok: false, source: 'Computer clock', offset_s: 0};
    var tsTxt = ts.ok ? 'GPS time (' + (ts.offset_s >= 0 ? '+' : '') + ts.offset_s + ' s)'
                      : 'Computer clock';
    pill($('stClock'), tsTxt, ts.ok ? 'ok' : 'bad');

    // device indicators
    var d = s.device;
    pill($('stWork'), d.work_mode, d.work_mode === 'Working Normally' ? 'ok' : '');
    pill($('stPps'), d.pps, (/sync|ok/i.test(d.pps) && !/no |error/i.test(d.pps)) ? 'ok' : '');
    pill($('stTemp'), d.temperature, cls(d.temperature));
    pill($('stVolt'), d.voltage, cls(d.voltage));
    pill($('stMotor'), d.motor, cls(d.motor));
    pill($('stDust'), d.dust, /clean|not dirty|normal/i.test(d.dust) ? 'ok' : (d.dust === 'Unknown' ? '' : 'warn'));
    pill($('stLife'), d.service_life, /ok|normal/i.test(d.service_life) ? 'ok' : (d.service_life === 'Unknown' ? '' : 'warn'));

    // USB
    var u = s.usb;
    $('usbLabel').textContent = u.mounted ? (u.label || 'USB') + ' ' + u.total_gb + ' GB' : 'No drive';
    pill($('usbState'), u.mounted ? 'Mounted' : (u.present ? 'Not mounted' : 'Not attached'),
         u.mounted ? 'ok' : 'bad');
    $('usbFree').textContent = u.mounted ? (u.free_pct + '% free - ' + u.free_gb + ' GB of ' + u.total_gb + ' GB') : '—';
    $('usbBar').style.width = (u.mounted ? u.free_pct : 0) + '%';

    // reflect the applied config into the dropdowns (only when not being edited)
    var c = s.config || {};
    setSel('cfgEcho', c.echo_type);
    setSel('cfgWork', c.work_mode);
    setSel('cfgImu', c.imu_freq);
    setSel('cfgScan', c.scan_mode);
    setSel('cfgCoord', c.coordinate);
    setSel('cfgHiSens', c.high_sensitivity);
    setSel('cfgRtk', c.rtk_source);
  }

  // Config <select>s the user has changed but not yet applied. We must NOT
  // overwrite these on the ~2 Hz status refresh, or the dropdown snaps back
  // before they can hit APPLY. Cleared when they apply (the value then matches).
  var edited = {};

  // Set a <select> to a value unless the user is picking in it or has an
  // unsaved change pending.
  function setSel(id, val) {
    var el = $(id);
    if (el && val != null && !edited[id] && document.activeElement !== el) {
      el.value = val;
    }
  }

  function cls(v) {
    if (!v || v === 'Unknown') return '';
    return /normal|ok/i.test(v) ? 'ok' : 'warn';
  }
  function fmtTime(sec) {
    var m = Math.floor(sec / 60), s = sec % 60;
    return m + ':' + (s < 10 ? '0' : '') + s;
  }

  // ---- wire up ----------------------------------------------------------
  $('btnStart').onclick = function () { post('/api/logging/start'); };
  $('btnStop').onclick = function () { post('/api/logging/stop'); };
  $('btnRestart').onclick = function () {
    confirmPost('/api/system/restart', 'Restart the Pi now?');
  };
  $('btnShutdown').onclick = function () {
    confirmPost('/api/system/shutdown', 'Shut the Pi down now?');
  };
  // Mark a dropdown as user-edited the moment they change it, so the live
  // refresh stops overwriting it until they apply.
  var cfgSelects = ['cfgEcho', 'cfgWork', 'cfgImu', 'cfgScan', 'cfgCoord',
                    'cfgHiSens', 'cfgRtk'];
  cfgSelects.forEach(function (id) {
    var el = $(id);
    if (el) { el.addEventListener('change', function () { edited[id] = true; }); }
  });
  function clearEdited(ids) { ids.forEach(function (id) { edited[id] = false; }); }

  $('btnApply').onclick = function () {
    var ids = ['cfgEcho', 'cfgWork', 'cfgImu', 'cfgScan', 'cfgCoord', 'cfgHiSens'];
    post('/api/config', {
      echo_type: $('cfgEcho').value,
      work_mode: $('cfgWork').value,
      imu_freq: $('cfgImu').value,
      scan_mode: $('cfgScan').value,
      coordinate: $('cfgCoord').value,
      high_sensitivity: $('cfgHiSens').value
    }).then(function () { clearEdited(ids); });
  };
  $('btnApplyRtk').onclick = function () {
    post('/api/config', { rtk_source: $('cfgRtk').value })
      .then(function () { clearEdited(['cfgRtk']); });
  };
  $('btnUsbAttach').onclick = function () { post('/api/usb/attach'); };
  $('btnUsbDetach').onclick = function () {
    confirmPost('/api/usb/detach', 'Safely eject the USB drive?');
  };
  $('btnUsbFormat').onclick = function () {
    confirmPost('/api/usb/format', 'ERASE everything on the USB drive? This cannot be undone.');
  };

  // ---- live stream ------------------------------------------------------
  function connect() {
    try {
      var es = new EventSource('/api/events');
      es.onmessage = function (ev) {
        try { render(JSON.parse(ev.data)); } catch (e) {}
      };
      es.onerror = function () { es.close(); setTimeout(connect, 2000); };
    } catch (e) {
      // fallback: poll
      setInterval(function () {
        fetch('/api/status').then(function (r) { return r.json(); }).then(render);
      }, 1000);
    }
  }
  connect();
})();
