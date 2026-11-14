# Head-estimator assets

Google MediaPipe Face Mesh V2 and BlazeFace short range model weights are licensed
under Apache-2.0 according to Google's model cards. The cards are kept locally as
licence evidence and listed in MANIFEST.json. They are not redistributed, because
they embed photographs (2026-10-18). Originals:
- https://storage.googleapis.com/mediapipe-assets/MediaPipe%20BlazeFace%20Model%20Card%20%28Short%20Range%29.pdf
- https://storage.googleapis.com/mediapipe-assets/Model%20Card%20MediaPipe%20Face%20Mesh%20V2.pdf Downloaded from the version-1
Google face_landmarker bundle identified in MANIFEST.json. The two TFLite files are
unchanged. The blendshape network and task geometry binary are not used or bundled.

Canonical geometry and landmark weights: Copyright 2020 The MediaPipe Authors,
Apache-2.0, source tag v0.10.21:
- https://github.com/google-ai-edge/mediapipe/blob/v0.10.21/mediapipe/modules/face_geometry/data/canonical_face_model.obj
- https://github.com/google-ai-edge/mediapipe/blob/v0.10.21/mediapipe/modules/face_geometry/data/geometry_pipeline_metadata_landmarks.pbtxt

The derived geometry_data.rs selects the 33 weighted vertices and changes basis to
(x,-y,-z). Its Apache attribution is retained. Our weak-perspective rotation fit is
not MediaPipe's metric face-geometry calculator; no equivalence or metric depth is
claimed. Detector decoding/crop constants were checked against pinned Google graphs:
- https://github.com/google-ai-edge/mediapipe/blob/v0.10.21/mediapipe/tasks/cc/vision/face_detector/face_detector_graph.cc
- https://github.com/google-ai-edge/mediapipe/blob/v0.10.21/mediapipe/tasks/cc/vision/face_landmarker/face_landmarks_detector_graph.cc

No community implementation, SCRFD or 6DRepNet weights are adopted for this candidate.
No third-party model training is performed. See LICENSE-MEDIAPIPE and MANIFEST.json.
