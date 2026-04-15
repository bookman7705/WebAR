import { initCamera, getVideo } from './camera.js';
import { initTracking, processTrackingFrame } from './tracking.js?v=tracking-fix-5';
import { drawTracking, drawFeaturePointsOverlay } from './visualization.js?v=canvas2-overlay-1';
import {
  initUI,
  updateHUD,
  isDebugDrawEnabled,
  isHomographyDebugEnabled,
  consumeRedetectRequest
} from './ui.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const canvas2 = document.getElementById('canvas2');
const ctx2 = canvas2 ? canvas2.getContext('2d') : null;

/** Set true to draw a lime test square at (100,100) and periodic console logs */
const DEBUG_CANVAS2_OVERLAY = false;

let loopStarted = false;
let canvas2DebugFrame = 0;

function syncCanvas2ToVideo(video) {
  if (!canvas2 || !video || video.videoWidth <= 0 || video.videoHeight <= 0) {
    return;
  }
  if (canvas2.width !== video.videoWidth || canvas2.height !== video.videoHeight) {
    canvas2.width = video.videoWidth;
    canvas2.height = video.videoHeight;
  }
}

function startRenderLoop() {
  if (loopStarted) {
    return;
  }
  loopStarted = true;
  processFrame();
}

function processFrame() {
  const video = getVideo();
  if (!video) {
    requestAnimationFrame(processFrame);
    return;
  }

  if (video.readyState === video.HAVE_ENOUGH_DATA) {
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;

    syncCanvas2ToVideo(video);

    ctx.drawImage(video, 0, 0);

    const trackingData = processTrackingFrame(canvas, {
      forceRedetect: consumeRedetectRequest()
    });

    drawTracking(
      ctx,
      trackingData,
      isDebugDrawEnabled(),
      isHomographyDebugEnabled()
    );

    if (ctx2 && canvas2 && canvas2.width > 0 && canvas2.height > 0) {
      ctx2.clearRect(0, 0, canvas2.width, canvas2.height);
      if (DEBUG_CANVAS2_OVERLAY) {
        ctx2.fillStyle = 'lime';
        ctx2.fillRect(100, 100, 10, 10);
        canvas2DebugFrame += 1;
        if (canvas2DebugFrame % 30 === 1) {
          const pts = trackingData.trackedPoints || [];
          console.log('points:', pts);
          console.log('canvas2 size:', canvas2.width, canvas2.height);
        }
      }
      drawFeaturePointsOverlay(
        ctx2,
        trackingData,
        video,
        isDebugDrawEnabled()
      );
    }

    updateHUD(trackingData.trackedCount, trackingData.status);
  }

  requestAnimationFrame(processFrame);
}

async function initApp() {
  initUI();
  await initCamera();
  await initTracking(getVideo());

  const video = getVideo();
  if (!video) {
    return;
  }

  // Mobile browsers can fire play before listeners are attached, so we use
  // multiple readiness hooks and an immediate readyState check.
  video.addEventListener('play', startRenderLoop);
  video.addEventListener('playing', startRenderLoop);
  video.addEventListener('loadeddata', startRenderLoop);
  video.addEventListener('loadedmetadata', () => {
    syncCanvas2ToVideo(video);
    startRenderLoop();
  });

  if (video.readyState >= video.HAVE_CURRENT_DATA) {
    syncCanvas2ToVideo(video);
    startRenderLoop();
  }
}

initApp().catch((error) => {
  console.error('App initialization failed:', error);
});
