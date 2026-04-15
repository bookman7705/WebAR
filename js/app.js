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

let loopStarted = false;

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
    const vw = video.videoWidth;
    const vh = video.videoHeight;

    if (vw > 0 && vh > 0) {
      canvas.width = vw;
      canvas.height = vh;

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

      // Draw on the same buffer as the video (1:1 with tracking coordinates).
      drawFeaturePointsOverlay(ctx, trackingData, video, isDebugDrawEnabled());

      updateHUD(trackingData.trackedCount, trackingData.status);
    }
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
  video.addEventListener('loadedmetadata', startRenderLoop);

  if (video.readyState >= video.HAVE_CURRENT_DATA) {
    startRenderLoop();
  }
}

initApp().catch((error) => {
  console.error('App initialization failed:', error);
});
