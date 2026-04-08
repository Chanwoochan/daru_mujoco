#include "daru_mujoco/teleop/daru_v4_mujoco_teleop_gui.hpp"

#include <GLFW/glfw3.h>
#include <cairo/cairo.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "daru_mujoco/package_paths.hpp"

namespace daru_mj {
namespace {

enum PanelId {
  kPanelStatus = 0,
  kPanelScene,
  kPanelKeys,
  kPanelJoints,
  kPanelActuators,
  kPanelCount
};

struct PanelState {
  bool visible = true;
  bool user_moved = false;
  int left = 0;
  int top = 0;
};

struct PanelStyle {
  std::array<float, 4> title_rgba;
  std::array<float, 4> body_rgba;
};

struct HudLayout {
  mjrRect toolbar;
  mjrRect status;
  mjrRect scene;
  mjrRect controls;
  mjrRect joints;
  mjrRect actuators;
};

struct PanelUi {
  mjrRect rect;
  mjrRect header;
  mjrRect toggle;
};

struct ToolbarUi {
  mjrRect rect;
  std::array<mjrRect, kPanelCount> buttons{};
};

struct WindowUi {
  HudLayout layout;
  ToolbarUi toolbar;
  std::array<PanelUi, kPanelCount> panels{};
};

constexpr std::array<int, kPanelCount> kDefaultPanelOrder = {kPanelStatus, kPanelScene, kPanelKeys, kPanelJoints,
                                                              kPanelActuators};
constexpr std::array<const char*, kPanelCount> kPanelTitles = {"Status", "Scene", "Keys", "Joints", "Actuators"};
constexpr std::array<const char*, kPanelCount> kPanelToggleLabels = {"STAT", "SCENE", "KEYS", "JOINTS", "ACTS"};

enum CategoryId {
  kCategoryFavorites = 0,
  kCategoryStatus,
  kCategoryScene,
  kCategoryKeys,
  kCategoryJoints,
  kCategoryActuators,
  kCategoryCount
};

struct DrawerItem {
  int source_category = kCategoryStatus;
  int item_index = -1;
  bool can_favorite = false;
  bool favorite = false;
  std::array<std::string, 6> columns{};
  int column_count = 0;
  std::string label;
  std::string value;
};

struct DrawerEntryUi {
  DrawerItem item;
  mjrRect rect;
  mjrRect favorite_button;
};

struct NavigationUi {
  mjrRect dock_rect;
  mjrRect drawer_rect;
  mjrRect header_rect;
  mjrRect title_rect;
  mjrRect category_rect;
  mjrRect columns_rect;
  mjrRect body_rect;
  mjrRect footer_rect;
  std::array<mjrRect, kCategoryCount> dock_buttons{};
  std::array<mjrRect, 4> action_buttons{};
  mjrRect prev_button;
  mjrRect next_button;
  int page_index = 0;
  int total_pages = 1;
  int total_items = 0;
  int range_start = 0;
  int range_end = 0;
  std::vector<DrawerEntryUi> entries;
};

struct DockUi {
  mjrRect rect;
  std::array<mjrRect, kCategoryCount> buttons{};
};

struct FloatingWindowState {
  bool visible = false;
  bool minimized = false;
  bool user_moved = false;
  int left = 0;
  int top = 0;
};

struct FloatingWindowUi {
  int category = kCategoryStatus;
  bool visible = false;
  bool minimized = false;
  mjrRect rect{};
  mjrRect header_rect{};
  mjrRect title_rect{};
  mjrRect subtitle_rect{};
  mjrRect icon_rect{};
  mjrRect close_button{};
  mjrRect minimize_button{};
  mjrRect columns_rect{};
  mjrRect body_rect{};
  mjrRect footer_rect{};
  mjrRect favorite_filter_button{};
  mjrRect prev_button{};
  mjrRect next_button{};
  mjrRect pause_button{};
  mjrRect step_button{};
  mjrRect task_reset_button{};
  mjrRect reset_button{};
  mjrRect graph_button{};
  int page_index = 0;
  int total_pages = 1;
  int total_items = 0;
  int range_start = 0;
  int range_end = 0;
  bool show_favorites_button = false;
  bool show_graph_button = false;
  bool show_page_controls = false;
  bool show_status_actions = false;
  std::vector<DrawerEntryUi> entries;
};

struct GraphWindowState {
  bool visible = true;
  bool minimized = false;
  bool user_moved = false;
  bool show_selector = true;
  bool y_auto = true;
  int left = 0;
  int top = 0;
  int serial = 0;
  int selected_joint = 0;
  int joint_page = 0;
  double x_seconds = 5.0;
  double y_manual_limit = 2.0;
};

struct GraphWindowUi {
  int graph_index = -1;
  int serial = 0;
  bool visible = false;
  bool minimized = false;
  bool show_selector = true;
  bool y_auto = true;
  mjrRect rect{};
  mjrRect header_rect{};
  mjrRect controls_rect{};
  mjrRect title_rect{};
  mjrRect subtitle_rect{};
  mjrRect icon_rect{};
  mjrRect close_button{};
  mjrRect minimize_button{};
  mjrRect selector_toggle_button{};
  mjrRect y_mode_button{};
  mjrRect y_value_button{};
  mjrRect y_minus_button{};
  mjrRect y_plus_button{};
  mjrRect x_value_button{};
  mjrRect x_minus_button{};
  mjrRect x_plus_button{};
  mjrRect selector_rect{};
  mjrRect selector_header_rect{};
  mjrRect selector_body_rect{};
  mjrRect selector_footer_rect{};
  mjrRect prev_button{};
  mjrRect next_button{};
  std::array<mjrRect, 3> graph_rects{};
  int page_index = 0;
  int total_pages = 1;
  int range_start = 0;
  int range_end = 0;
  int selected_joint = 0;
  double x_seconds = 5.0;
  double y_manual_limit = 2.0;
  std::vector<int> visible_joint_ids;
  std::vector<mjrRect> joint_rows;
};

struct DesktopUi {
  DockUi dock;
  std::array<FloatingWindowUi, kCategoryCount> windows{};
  std::vector<GraphWindowUi> graph_windows;
};

constexpr std::array<const char*, kCategoryCount> kCategoryTitles = {"Favorites", "Status", "Scene", "Keys", "Joints",
                                                                     "Actuators"};
constexpr std::array<const char*, kCategoryCount> kCategoryDockLabels = {"FAV", "STAT", "SCENE", "KEYS", "JOINTS",
                                                                         "ACTS"};
constexpr std::array<const char*, kCategoryCount> kCategoryIcons = {"*", "=", "O", ">", "J", "A"};
constexpr std::array<int, kCategoryCount> kDefaultWindowOrder = {
    kCategoryFavorites, kCategoryStatus, kCategoryScene, kCategoryKeys, kCategoryJoints, kCategoryActuators};

enum ActionButtonId {
  kActionPause = 0,
  kActionStep,
  kActionReset,
  kActionFavoritesOnly,
  kActionCount
};

enum UiIconId {
  kIconFavorites = 0,
  kIconStatus,
  kIconScene,
  kIconKeys,
  kIconJoints,
  kIconActuators,
  kIconFavoriteOn,
  kIconFavoriteOff,
  kIconCount
};

struct UiIconTexture {
  int width = 0;
  int height = 0;
  std::vector<unsigned char> bgra;
};

constexpr int kJointHistorySamples = 2400;

struct JointHistory {
  std::array<float, kJointHistorySamples> q{};
  std::array<float, kJointHistorySamples> v{};
  std::array<float, kJointHistorySamples> tau{};
};

constexpr std::array<double, 8> kGraphXSecondsPresets = {0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0};
constexpr std::array<double, 10> kGraphYLimitPresets = {0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};

enum GraphEditField {
  kGraphEditNone = 0,
  kGraphEditXSeconds,
  kGraphEditYLimit
};

constexpr std::array<const char*, kIconCount> kIconFilenames = {
    "favorites.png",
    "status.png",
    "scene.png",
    "keys.png",
    "joints.png",
    "actuators.png",
    "favorite_on.png",
    "favorite_off.png",
};

struct KeyRowDef {
  const char* label;
  const char* value;
  bool compact_visible;
};

constexpr std::array<KeyRowDef, 10> kKeyRows = {{
    {"Help", "H", true},
    {"Pages", "PGUP / PGDN", true},
    {"Favorite", "click [ ]", true},
    {"Category", "dock click", true},
    {"Pause", "SPACE", false},
    {"Step", "RIGHT", false},
    {"Reset", "R", false},
    {"Visual", "C J A W S F", false},
    {"Speed", "- / +", false},
    {"Perturb", "Ctrl + Drag", false},
}};

struct ViewerState {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  DaruV4MuJoCoController* controller = nullptr;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjvScene offscreen_scn;
  mjrContext con;
  mjvPerturb pert;

  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0.0;
  double lasty = 0.0;

  bool paused = false;
  bool request_single_step = false;
  bool show_help = false;
  bool slow_motion = false;
  bool ui_capture = false;

  double realtime_scale = 1.0;
  int camera_mode = 0;  // 0: free, 1..ncam: fixed camera index+1
  int joint_page = 0;
  int actuator_page = 0;
  int font_scale = mjFONTSCALE_100;
  int step_count = 0;
  int active_category = kCategoryJoints;
  int focused_graph_window = -1;
  int editing_graph_window = -1;
  int editing_graph_field = kGraphEditNone;
  int drag_window = -1;
  int drag_graph_window = -1;
  int drag_panel = -1;
  double drag_offset_x = 0.0;
  double drag_offset_y = 0.0;
  std::array<PanelState, kPanelCount> panels{};
  std::array<int, kPanelCount> panel_order = kDefaultPanelOrder;
  std::array<FloatingWindowState, kCategoryCount> floating_windows{};
  std::array<int, kCategoryCount> window_order = kDefaultWindowOrder;
  std::vector<GraphWindowState> graph_windows;
  std::vector<int> graph_window_order;
  int next_graph_window_serial = 1;
  std::array<int, kCategoryCount> category_pages{};
  std::array<bool, kCategoryCount> favorites_only{};
  std::array<bool, 6> favorite_status{};
  std::array<bool, 6> favorite_scene{};
  std::array<bool, kKeyRows.size()> favorite_keys{};
  std::vector<unsigned char> favorite_joints;
  std::vector<unsigned char> favorite_actuators;
  std::vector<JointHistory> joint_history;
  std::array<float, kJointHistorySamples> joint_history_time{};
  int joint_history_cursor = 0;
  int joint_history_count = 0;
  double last_history_time = -1.0;
  std::array<UiIconTexture, kIconCount> icons{};
  bool icons_loaded = false;
  int last_viewport_width = 0;
  int last_viewport_height = 0;
  std::string graph_edit_buffer;
  int cloth_body_id = -1;
  int cloth_flex_id = -1;
  std::array<mjtNum, 3> cloth_default_pos{};
  std::array<mjtNum, 4> cloth_default_quat{1.0, 0.0, 0.0, 0.0};
  std::array<float, 4> cloth_default_rgba{0.75f, 0.20f, 0.20f, 1.0f};
  std::array<int, 2> task_box_body_ids{{-1, -1}};
  std::array<int, 2> task_box_geom_ids{{-1, -1}};
  std::array<int, 2> task_box_qpos_adrs{{-1, -1}};
  std::array<std::array<mjtNum, 7>, 2> task_box_default_qpos0{};
  std::array<mjtNum, 2> task_box_default_mass{};
  std::array<std::array<mjtNum, 3>, 2> task_box_default_inertia{};
  std::array<std::array<float, 4>, 2> task_box_default_rgba{};
  int task_tray_body_id = -1;
  std::array<mjtNum, 3> task_tray_default_pos{};
  std::array<mjtNum, 4> task_tray_default_quat{1.0, 0.0, 0.0, 0.0};
  std::mt19937 cloth_rng{std::random_device{}()};
};

ViewerState* g_viewer = nullptr;

void PublishStereoFromGui(ViewerState& viewer,
                          DaruV4MuJoCoRosBridge& ros_bridge,
                          int width,
                          int height);


const PanelStyle kPanelStyle = {{{0.12f, 0.43f, 0.57f, 0.92f}}, {{0.02f, 0.05f, 0.08f, 0.42f}}};

constexpr std::array<mjtNum, 3> kDefaultFreeCameraLookat = {0.58, 0.0, 0.95};
constexpr mjtNum kDefaultFreeCameraDistance = 2.15;
constexpr mjtNum kDefaultFreeCameraAzimuth = 142.0;
constexpr mjtNum kDefaultFreeCameraElevation = -32.0;

void UpdateUiScale(ViewerState& viewer, int width, int height);
void DrawHud(const ViewerState& viewer, const DaruV4MuJoCoController& controller, const mjrRect& viewport,
             int step_count);

void ApplyDefaultFreeCameraPose(mjvCamera& cam) {
  cam.type = mjCAMERA_FREE;
  cam.fixedcamid = -1;
  cam.lookat[0] = kDefaultFreeCameraLookat[0];
  cam.lookat[1] = kDefaultFreeCameraLookat[1];
  cam.lookat[2] = kDefaultFreeCameraLookat[2];
  cam.distance = kDefaultFreeCameraDistance;
  cam.azimuth = kDefaultFreeCameraAzimuth;
  cam.elevation = kDefaultFreeCameraElevation;
}

char FlagChar(bool enabled) { return enabled ? 'Y' : 'N'; }

std::string SafeName(const mjModel* model, mjtObj type, int id, const std::string& fallback_prefix = "item") {
  if (!model || id < 0) {
    return fallback_prefix + " ?";
  }
  const char* name = mj_id2name(model, type, id);
  if (name && name[0] != '\0') {
    return name;
  }
  std::ostringstream oss;
  oss << fallback_prefix << " " << id;
  return oss.str();
}

std::string TruncateText(const std::string& text, std::size_t max_len) {
  if (text.size() <= max_len || max_len < 4) {
    return text;
  }
  return text.substr(0, max_len - 3) + "...";
}

std::string FormatFixed(double value, int precision = 3) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return buffer;
}

std::string FormatSigned(double value, int precision = 3) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%+.*f", precision, value);
  return buffer;
}

int JointQposCount(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 7;
    case mjJNT_BALL:
      return 4;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
    default:
      return 0;
  }
}

int JointDofCount(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 6;
    case mjJNT_BALL:
      return 3;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
    default:
      return 0;
  }
}

const char* JointTypeName(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return "free";
    case mjJNT_BALL:
      return "ball";
    case mjJNT_SLIDE:
      return "slide";
    case mjJNT_HINGE:
      return "hinge";
    default:
      return "joint";
  }
}

const char* JointTypeShortName(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return "F";
    case mjJNT_BALL:
      return "B";
    case mjJNT_SLIDE:
      return "S";
    case mjJNT_HINGE:
      return "H";
    default:
      return "?";
  }
}

const char* TransmissionName(int trn_type) {
  switch (trn_type) {
    case mjTRN_JOINT:
      return "joint";
    case mjTRN_JOINTINPARENT:
      return "joint-parent";
    case mjTRN_SLIDERCRANK:
      return "slider";
    case mjTRN_TENDON:
      return "tendon";
    case mjTRN_SITE:
      return "site";
    case mjTRN_BODY:
      return "body";
    default:
      return "other";
  }
}

std::string ActuatorTargetName(const mjModel* model, int actuator_id) {
  const int trn_type = model->actuator_trntype[actuator_id];
  const int trn_id0 = model->actuator_trnid[2 * actuator_id];
  switch (trn_type) {
    case mjTRN_JOINT:
    case mjTRN_JOINTINPARENT:
      return SafeName(model, mjOBJ_JOINT, trn_id0, "joint");
    case mjTRN_TENDON:
      return SafeName(model, mjOBJ_TENDON, trn_id0, "tendon");
    case mjTRN_SITE:
      return SafeName(model, mjOBJ_SITE, trn_id0, "site");
    case mjTRN_BODY:
      return SafeName(model, mjOBJ_BODY, trn_id0, "body");
    default:
      return TransmissionName(trn_type);
  }
}

std::string ActuatorModeName(const mjModel* model, int actuator_id) {
  const int dyn_type = model->actuator_dyntype[actuator_id];
  const int gain_type = model->actuator_gaintype[actuator_id];
  const int bias_type = model->actuator_biastype[actuator_id];

  if (dyn_type == mjDYN_NONE && gain_type == mjGAIN_FIXED && bias_type == mjBIAS_NONE) {
    return "motor";
  }
  if (dyn_type == mjDYN_NONE && gain_type == mjGAIN_FIXED && bias_type == mjBIAS_AFFINE) {
    return "servo";
  }
  if (gain_type == mjGAIN_AFFINE) {
    return "affine";
  }
  if (dyn_type != mjDYN_NONE) {
    return "dynamic";
  }
  return "custom";
}

std::string CurrentCameraLabel(const ViewerState& viewer) {
  if (viewer.cam.type != mjCAMERA_FIXED || viewer.cam.fixedcamid < 0) {
    return "FREE";
  }
  std::ostringstream oss;
  oss << "FIXED: " << TruncateText(SafeName(viewer.model, mjOBJ_CAMERA, viewer.cam.fixedcamid, "cam"), 18);
  return oss.str();
}

std::string FrameModeLabel(int frame_mode) {
  switch (frame_mode) {
    case mjFRAME_BODY:
      return "BODY";
    case mjFRAME_GEOM:
      return "GEOM";
    case mjFRAME_SITE:
      return "SITE";
    case mjFRAME_WORLD:
      return "WORLD";
    default:
      return "NONE";
  }
}

std::string SelectedBodyLabel(const ViewerState& viewer) {
  if (viewer.pert.select <= 0) {
    return "none";
  }
  std::ostringstream oss;
  oss << viewer.pert.select << " / "
      << TruncateText(SafeName(viewer.model, mjOBJ_BODY, viewer.pert.select, "body"), 18);
  return oss.str();
}

int PanelTitleHeight(const mjrContext& context) { return std::clamp(context.charHeight + 8, 18, 30); }

int PanelBodyPadX(const mjrContext& context) { return std::clamp(context.charHeight / 3, 4, 10); }

int PanelBodyPadY(const mjrContext& context) { return std::clamp(context.charHeight / 3, 4, 10); }

int PanelLineHeight(const mjrContext& context) {
  return std::max(1, context.charHeight + std::clamp(context.charHeight / 6, 3, 8));
}

int PanelHeightForLines(const mjrContext& context, int line_count) {
  const int lines = std::max(1, line_count);
  return PanelTitleHeight(context) + 2 * PanelBodyPadY(context) + lines * PanelLineHeight(context);
}

int EstimateGlyphWidth(const mjrContext& context, char ch) {
  const int base = std::max(5, context.charHeight * 11 / 20);
  switch (ch) {
    case ' ':
      return std::max(3, base / 2);
    case '.':
    case ',':
    case ':':
    case ';':
      return std::max(2, base / 2);
    case '+':
    case '-':
      return std::max(4, base * 2 / 3);
    case 'I':
    case 'l':
    case '1':
    case '[':
    case ']':
      return std::max(4, base * 2 / 3);
    case 'M':
    case 'W':
    case '@':
    case '#':
      return base * 5 / 4;
    default:
      return base;
  }
}

int EstimateTextWidth(const mjrContext& context, const std::string& text) {
  int width = 0;
  for (char ch : text) {
    width += EstimateGlyphWidth(context, ch);
  }
  return width;
}

int ToolbarHeight(const mjrContext& context) { return std::clamp(context.charHeight + 14, 24, 40); }

int DesiredToolbarWidth(const mjrContext& context) {
  const int button_gap = std::clamp(context.charHeight / 3, 4, 10);
  const int button_pad = std::clamp(context.charHeight / 2, 8, 18);
  int total = 2 * button_pad + button_gap * (static_cast<int>(kPanelCount) - 1);
  for (const char* label : kPanelToggleLabels) {
    total += std::max(48, EstimateTextWidth(context, label) + button_pad);
  }
  return total;
}

std::array<std::filesystem::path, 5> TextureSearchBases() {
  const auto dynamic_bases = daru_mj::GetKcwarmMujocoTextureSearchBases();
  std::array<std::filesystem::path, 5> bases{};
  for (size_t i = 0; i < bases.size(); ++i) {
    bases[i] = (i < dynamic_bases.size()) ? dynamic_bases[i] : std::filesystem::path();
  }
  return bases;
}

std::filesystem::path ResolveTexturePath(const std::filesystem::path& relative_path) {
  for (const auto& base : TextureSearchBases()) {
    const std::filesystem::path candidate = base / relative_path;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return {};
}

bool HasPngExtension(const std::filesystem::path& path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return ext == ".png";
}

std::filesystem::path ResolveUiIconPath(const char* filename) {
  return ResolveTexturePath(std::filesystem::path("ui") / filename);
}

std::filesystem::path ResolveStartupLogoPath() {
  for (const char* preferred : {"logo.png", "rclab.png", "splash.png"}) {
    const std::filesystem::path candidate = ResolveTexturePath(std::filesystem::path("logo") / preferred);
    if (!candidate.empty()) {
      return candidate;
    }
  }

  for (const auto& base : TextureSearchBases()) {
    const std::filesystem::path logo_dir = base / "logo";
    if (!std::filesystem::exists(logo_dir) || !std::filesystem::is_directory(logo_dir)) {
      continue;
    }

    std::vector<std::filesystem::path> pngs;
    for (const auto& entry : std::filesystem::directory_iterator(logo_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      if (HasPngExtension(entry.path())) {
        pngs.push_back(entry.path());
      }
    }
    std::sort(pngs.begin(), pngs.end());
    if (!pngs.empty()) {
      return pngs.front();
    }
  }

  std::array<std::filesystem::path, 5> bases{};
  const auto texture_bases = TextureSearchBases();
  for (size_t i = 0; i < bases.size(); ++i) {
    if (!texture_bases[i].empty()) {
      bases[i] = texture_bases[i] / "logo";
    }
  }
  for (const auto& base : bases) {
    if (!std::filesystem::exists(base) || !std::filesystem::is_directory(base)) {
      continue;
    }
    for (const auto& entry : std::filesystem::directory_iterator(base)) {
      if (entry.is_regular_file() && HasPngExtension(entry.path())) {
        return entry.path();
      }
    }
  }
  return {};
}

bool LoadIconTexture(UiIconTexture& icon, const char* path) {
  cairo_surface_t* surface = cairo_image_surface_create_from_png(path);
  if (!surface || cairo_surface_status(surface) != CAIRO_STATUS_SUCCESS) {
    if (surface) {
      cairo_surface_destroy(surface);
    }
    return false;
  }

  cairo_surface_flush(surface);
  const int width = cairo_image_surface_get_width(surface);
  const int height = cairo_image_surface_get_height(surface);
  const int stride = cairo_image_surface_get_stride(surface);
  unsigned char* data = cairo_image_surface_get_data(surface);
  if (!data || width <= 0 || height <= 0) {
    cairo_surface_destroy(surface);
    return false;
  }

  icon.width = width;
  icon.height = height;
  icon.bgra.assign(static_cast<std::size_t>(width * height * 4), 0);
  for (int y = 0; y < height; ++y) {
    std::copy_n(data + y * stride, width * 4, icon.bgra.begin() + static_cast<std::size_t>(y * width * 4));
  }
  cairo_surface_destroy(surface);
  return true;
}

void EnsureUiIconsLoaded(ViewerState& viewer) {
  if (viewer.icons_loaded) {
    return;
  }
  for (std::size_t i = 0; i < viewer.icons.size(); ++i) {
    const std::filesystem::path icon_path = ResolveUiIconPath(kIconFilenames[i]);
    if (icon_path.empty()) {
      std::cerr << "[DARU_MJ][UI] missing icon: " << kIconFilenames[i]
                << " (expected under resource/textures/ui)\n";
      continue;
    }
    LoadIconTexture(viewer.icons[i], icon_path.string().c_str());
  }
  viewer.icons_loaded = true;
}

void FreeUiIcons(ViewerState& viewer) {
  for (UiIconTexture& icon : viewer.icons) {
    icon.bgra.clear();
    icon.width = 0;
    icon.height = 0;
  }
  viewer.icons_loaded = false;
}

void DrawIconTexture(const UiIconTexture& icon, const mjrRect& rect, const mjrContext& context, float bg_r, float bg_g,
                     float bg_b, float alpha = 1.0f) {
  if (icon.bgra.empty() || rect.width <= 0 || rect.height <= 0 || icon.width <= 0 || icon.height <= 0) {
    return;
  }
  std::vector<unsigned char> rgb(static_cast<std::size_t>(rect.width * rect.height * 3), 0);
  const float bg_rf = std::clamp(bg_r, 0.0f, 1.0f);
  const float bg_gf = std::clamp(bg_g, 0.0f, 1.0f);
  const float bg_bf = std::clamp(bg_b, 0.0f, 1.0f);

  for (int dy = 0; dy < rect.height; ++dy) {
    const int flipped_dy = rect.height - 1 - dy;
    const int sy = std::clamp((flipped_dy * icon.height) / std::max(1, rect.height), 0, icon.height - 1);
    for (int dx = 0; dx < rect.width; ++dx) {
      const int sx = std::clamp((dx * icon.width) / std::max(1, rect.width), 0, icon.width - 1);
      const std::size_t src = static_cast<std::size_t>((sy * icon.width + sx) * 4);
      const float src_a = (static_cast<float>(icon.bgra[src + 3]) / 255.0f) * alpha;
      const float out_r = 1.0f * src_a + bg_rf * (1.0f - src_a);
      const float out_g = 1.0f * src_a + bg_gf * (1.0f - src_a);
      const float out_b = 1.0f * src_a + bg_bf * (1.0f - src_a);
      const std::size_t dst = static_cast<std::size_t>((dy * rect.width + dx) * 3);
      rgb[dst + 0] = static_cast<unsigned char>(std::clamp(out_r, 0.0f, 1.0f) * 255.0f);
      rgb[dst + 1] = static_cast<unsigned char>(std::clamp(out_g, 0.0f, 1.0f) * 255.0f);
      rgb[dst + 2] = static_cast<unsigned char>(std::clamp(out_b, 0.0f, 1.0f) * 255.0f);
    }
  }

  mjr_drawPixels(rgb.data(), nullptr, rect, &context);
}

void DrawImageTexture(const UiIconTexture& image, const mjrRect& rect, const mjrContext& context, float bg_r, float bg_g,
                      float bg_b, float alpha = 1.0f, bool key_black = false, bool composite_over_framebuffer = false) {
  if (image.bgra.empty() || rect.width <= 0 || rect.height <= 0 || image.width <= 0 || image.height <= 0) {
    return;
  }

  std::vector<unsigned char> rgb(static_cast<std::size_t>(rect.width * rect.height * 3), 0);
  const float bg_rf = std::clamp(bg_r, 0.0f, 1.0f);
  const float bg_gf = std::clamp(bg_g, 0.0f, 1.0f);
  const float bg_bf = std::clamp(bg_b, 0.0f, 1.0f);

  if (composite_over_framebuffer) {
    mjr_readPixels(rgb.data(), nullptr, rect, &context);
  } else {
    for (std::size_t i = 0; i < rgb.size(); i += 3) {
      rgb[i + 0] = static_cast<unsigned char>(bg_rf * 255.0f);
      rgb[i + 1] = static_cast<unsigned char>(bg_gf * 255.0f);
      rgb[i + 2] = static_cast<unsigned char>(bg_bf * 255.0f);
    }
  }

  for (int dy = 0; dy < rect.height; ++dy) {
    const int flipped_dy = rect.height - 1 - dy;
    const int sy = std::clamp((flipped_dy * image.height) / std::max(1, rect.height), 0, image.height - 1);
    for (int dx = 0; dx < rect.width; ++dx) {
      const int sx = std::clamp((dx * image.width) / std::max(1, rect.width), 0, image.width - 1);
      const std::size_t src = static_cast<std::size_t>((sy * image.width + sx) * 4);
      const float src_b = static_cast<float>(image.bgra[src + 0]) / 255.0f;
      const float src_g = static_cast<float>(image.bgra[src + 1]) / 255.0f;
      const float src_r = static_cast<float>(image.bgra[src + 2]) / 255.0f;
      float src_a = (static_cast<float>(image.bgra[src + 3]) / 255.0f) * alpha;
      if (key_black) {
        const float brightness = std::max({src_r, src_g, src_b});
        const float keyed_alpha = std::clamp((brightness - 0.04f) / 0.14f, 0.0f, 1.0f);
        src_a *= keyed_alpha;
      }
      const std::size_t dst = static_cast<std::size_t>((dy * rect.width + dx) * 3);
      const float dst_r = static_cast<float>(rgb[dst + 0]) / 255.0f;
      const float dst_g = static_cast<float>(rgb[dst + 1]) / 255.0f;
      const float dst_b = static_cast<float>(rgb[dst + 2]) / 255.0f;
      const float out_r = src_r * src_a + dst_r * (1.0f - src_a);
      const float out_g = src_g * src_a + dst_g * (1.0f - src_a);
      const float out_b = src_b * src_a + dst_b * (1.0f - src_a);
      rgb[dst + 0] = static_cast<unsigned char>(std::clamp(out_r, 0.0f, 1.0f) * 255.0f);
      rgb[dst + 1] = static_cast<unsigned char>(std::clamp(out_g, 0.0f, 1.0f) * 255.0f);
      rgb[dst + 2] = static_cast<unsigned char>(std::clamp(out_b, 0.0f, 1.0f) * 255.0f);
    }
  }

  mjr_drawPixels(rgb.data(), nullptr, rect, &context);
}

void DrawStartupSplashOverlay(const ViewerState& viewer, const UiIconTexture& logo, const mjrRect& viewport,
                              float overlay_alpha, bool show_title) {
  const float clamped_alpha = std::clamp(overlay_alpha, 0.0f, 1.0f);
  if (clamped_alpha <= 0.0f) {
    return;
  }

  const double max_logo_width = viewport.width * 0.42;
  const double max_logo_height = viewport.height * 0.42;
  const double scale =
      std::min(max_logo_width / std::max(1, logo.width), max_logo_height / std::max(1, logo.height));
  const int draw_width = std::max(1, static_cast<int>(std::lround(logo.width * scale)));
  const int draw_height = std::max(1, static_cast<int>(std::lround(logo.height * scale)));
  const int left = std::max(0, (viewport.width - draw_width) / 2);
  const int bottom = std::max(0, (viewport.height - draw_height) / 2 + viewer.con.charHeight);
  DrawImageTexture(logo, mjrRect{left, bottom, draw_width, draw_height}, viewer.con, 0.0f, 0.0f, 0.0f, clamped_alpha, true,
                   true);

  if (show_title) {
    const mjrRect title_rect = {0, bottom - viewer.con.charHeight - 24, viewport.width, viewer.con.charHeight + 8};
    const mjrRect subtitle_rect = {0, bottom - 2 * viewer.con.charHeight - 34, viewport.width, viewer.con.charHeight + 6};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOP, title_rect, "DARU MuJoCo Control Room", "", &viewer.con);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOP, subtitle_rect, "RCLAB", "", &viewer.con);
  }
}

void DrawStartupSplashOnly(GLFWwindow* window, ViewerState& viewer, const UiIconTexture& logo, float alpha, bool show_title) {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  UpdateUiScale(viewer, width, height);
  const mjrRect viewport = {0, 0, width, height};
  mjr_rectangle(viewport, 0.02f, 0.03f, 0.04f, 1.0f);
  DrawStartupSplashOverlay(viewer, logo, viewport, alpha, show_title);
}

void RenderSceneFrame(GLFWwindow* window,
                      ViewerState& viewer,
                      const DaruV4MuJoCoController& controller,
                      int step_count,
                      bool draw_hud = true) {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  UpdateUiScale(viewer, width, height);
  const mjrRect viewport = {0, 0, width, height};
  mjv_updateScene(viewer.model, viewer.data, &viewer.opt, &viewer.pert, &viewer.cam, mjCAT_ALL, &viewer.scn);
  mjr_render(viewport, &viewer.scn, &viewer.con);
  if (draw_hud) {
    DrawHud(viewer, controller, viewport, step_count);
  }
}

void ShowStartupSplash(GLFWwindow* window, ViewerState& viewer, const DaruV4MuJoCoController& controller) {
  UiIconTexture logo;
  const std::filesystem::path logo_path = ResolveStartupLogoPath();
  if (logo_path.empty() || !LoadIconTexture(logo, logo_path.string().c_str())) {
    return;
  }

  const double hold_seconds = 3.0;
  const double fade_seconds = 0.45;
  const double hold_start = glfwGetTime();
  while (!glfwWindowShouldClose(window) && glfwGetTime() - hold_start < hold_seconds) {
    DrawStartupSplashOnly(window, viewer, logo, 1.0f, true);
    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  const double fade_start = glfwGetTime();
  while (!glfwWindowShouldClose(window) && glfwGetTime() - fade_start < fade_seconds) {
    const double elapsed = glfwGetTime() - fade_start;
    const float alpha = static_cast<float>(std::clamp(1.0 - elapsed / fade_seconds, 0.0, 1.0));
    RenderSceneFrame(window, viewer, controller, 0, true);

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport = {0, 0, width, height};
    DrawStartupSplashOverlay(viewer, logo, viewport, alpha, false);

    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
}

template <std::size_t N>
std::array<int, N> FitStackHeights(int total_height, int gap, const std::array<int, N>& preferred, int min_height) {
  std::array<int, N> heights{};
  if constexpr (N == 0) {
    return heights;
  }

  const int available = std::max(0, total_height - gap * static_cast<int>(N - 1));
  if (available <= 0) {
    return heights;
  }

  int preferred_sum = 0;
  for (int value : preferred) {
    preferred_sum += std::max(1, value);
  }
  if (preferred_sum <= 0) {
    preferred_sum = static_cast<int>(N);
  }

  const int floor = std::min(min_height, available / static_cast<int>(N));
  const double scale = static_cast<double>(available) / preferred_sum;
  int assigned = 0;
  for (std::size_t i = 0; i < N; ++i) {
    heights[i] = std::max(floor, static_cast<int>(std::lround(std::max(1, preferred[i]) * scale)));
    assigned += heights[i];
  }

  while (assigned > available) {
    int best = -1;
    for (std::size_t i = 0; i < N; ++i) {
      if (heights[i] <= floor) {
        continue;
      }
      if (best < 0 || heights[i] > heights[best]) {
        best = static_cast<int>(i);
      }
    }
    if (best < 0) {
      break;
    }
    --heights[best];
    --assigned;
  }

  while (assigned < available) {
    std::size_t best = 0;
    for (std::size_t i = 1; i < N; ++i) {
      if (preferred[i] > preferred[best]) {
        best = i;
      }
    }
    ++heights[best];
    ++assigned;
  }

  return heights;
}

int PageCount(int total_items, int rows_per_page) {
  return std::max(1, (total_items + std::max(1, rows_per_page) - 1) / std::max(1, rows_per_page));
}

int DetermineFontScale(int width, int height) {
  const int min_dim = std::min(width, height);
  if (min_dim >= 2100 || width >= 3200) {
    return mjFONTSCALE_200;
  }
  if (min_dim >= 1600 || width >= 2600) {
    return mjFONTSCALE_150;
  }
  if (min_dim >= 980 || width >= 1700) {
    return mjFONTSCALE_100;
  }
  return mjFONTSCALE_50;
}

void UpdateUiScale(ViewerState& viewer, int width, int height) {
  const int desired_scale = DetermineFontScale(width, height);
  if (desired_scale == viewer.font_scale && viewer.con.glInitialized) {
    return;
  }

  if (viewer.con.glInitialized) {
    mjr_freeContext(&viewer.con);
  }

  mjr_makeContext(viewer.model, &viewer.con, desired_scale);
  viewer.font_scale = desired_scale;
}

mjrRect MakeTopLeftRect(const mjrRect& viewport, int left, int top, int width, int height) {
  return mjrRect{viewport.left + left, viewport.bottom + viewport.height - top - height, std::max(0, width),
                 std::max(0, height)};
}

mjrRect InsetRect(const mjrRect& rect, int pad_x, int pad_y) {
  return mjrRect{rect.left + pad_x, rect.bottom + pad_y, std::max(0, rect.width - 2 * pad_x),
                 std::max(0, rect.height - 2 * pad_y)};
}

int RectTop(const mjrRect& viewport, const mjrRect& rect) {
  return viewport.height - (rect.bottom + rect.height - viewport.bottom);
}

mjrRect DefaultPanelRect(const HudLayout& layout, int panel_id) {
  switch (panel_id) {
    case kPanelStatus:
      return layout.status;
    case kPanelScene:
      return layout.scene;
    case kPanelKeys:
      return layout.controls;
    case kPanelJoints:
      return layout.joints;
    case kPanelActuators:
      return layout.actuators;
    default:
      return mjrRect{0, 0, 0, 0};
  }
}

[[maybe_unused]] void BringPanelToFront(ViewerState& viewer, int panel_id) {
  auto it = std::find(viewer.panel_order.begin(), viewer.panel_order.end(), panel_id);
  if (it == viewer.panel_order.end()) {
    return;
  }
  const std::size_t index = static_cast<std::size_t>(std::distance(viewer.panel_order.begin(), it));
  for (std::size_t i = index; i + 1 < viewer.panel_order.size(); ++i) {
    viewer.panel_order[i] = viewer.panel_order[i + 1];
  }
  viewer.panel_order.back() = panel_id;
}

bool PointInRect(const mjrRect& viewport, const mjrRect& rect, double x, double y) {
  const int top = RectTop(viewport, rect);
  return x >= rect.left && x <= rect.left + rect.width && y >= top && y <= top + rect.height;
}

HudLayout ComputeHudLayout(const ViewerState& viewer, const mjrRect& viewport, const mjrContext& context) {
  const int min_dim = std::min(viewport.width, viewport.height);
  const int pad = std::clamp(min_dim / 115, 6, 14);
  const int gap = std::clamp(pad - 1, 4, 10);
  const int toolbar_height = ToolbarHeight(context);
  const int toolbar_width =
      std::min(std::max(220, DesiredToolbarWidth(context)), std::max(220, viewport.width - 2 * pad));

  const int content_top = pad + toolbar_height + gap;
  const int content_height = std::max(0, viewport.height - content_top - pad);
  const int free_width = std::max(0, viewport.width - 2 * pad - 2 * gap);
  const int min_center_width = std::clamp(viewport.width / 3, 220, 760);

  int left_width = std::clamp(free_width * 24 / 100, 180, 320);
  int right_width = std::clamp(free_width * 31 / 100, 240, 440);
  int center_width = std::max(0, free_width - left_width - right_width);
  if (center_width < min_center_width) {
    int deficit = min_center_width - center_width;
    const int right_take = std::min(deficit / 2, std::max(0, right_width - 220));
    right_width -= right_take;
    deficit -= right_take;

    const int left_take = std::min(deficit, std::max(0, left_width - 160));
    left_width -= left_take;
    deficit -= left_take;

    if (deficit > 0) {
      right_width = std::max(160, right_width - deficit);
    }
  }

  const int min_panel_height = PanelHeightForLines(context, 2);
  const std::array<int, 3> left_pref = {
      std::max(PanelHeightForLines(context, 6), content_height * 18 / 100),
      std::max(PanelHeightForLines(context, 6), content_height * 20 / 100),
      std::max(PanelHeightForLines(context, viewer.show_help ? 10 : 6), content_height * 62 / 100)};
  const std::array<int, 3> left_heights = FitStackHeights(content_height, gap, left_pref, min_panel_height);

  double joint_ratio = 0.58;
  const int total_items = viewer.model ? viewer.model->njnt + viewer.model->nu : 0;
  if (total_items > 0) {
    joint_ratio =
        std::clamp(static_cast<double>(viewer.model->njnt) / total_items, 0.48, 0.72);
  }
  const std::array<int, 2> right_pref = {
      std::max(PanelHeightForLines(context, 8), static_cast<int>(std::lround(content_height * joint_ratio))),
      std::max(PanelHeightForLines(context, 6),
               static_cast<int>(std::lround(content_height * (1.0 - joint_ratio))))};
  const std::array<int, 2> right_heights = FitStackHeights(content_height, gap, right_pref, min_panel_height);

  HudLayout layout{};
  layout.toolbar = MakeTopLeftRect(viewport, std::max(pad, viewport.width - pad - toolbar_width), pad, toolbar_width,
                                   toolbar_height);
  const int left_x = pad;
  const int right_x = std::max(pad, viewport.width - pad - right_width);

  int left_top = content_top;
  layout.status = MakeTopLeftRect(viewport, left_x, left_top, left_width, left_heights[0]);
  left_top += left_heights[0] + gap;
  layout.scene = MakeTopLeftRect(viewport, left_x, left_top, left_width, left_heights[1]);
  left_top += left_heights[1] + gap;
  layout.controls = MakeTopLeftRect(viewport, left_x, left_top, left_width, left_heights[2]);

  int right_top = content_top;
  layout.joints = MakeTopLeftRect(viewport, right_x, right_top, right_width, right_heights[0]);
  right_top += right_heights[0] + gap;
  layout.actuators = MakeTopLeftRect(viewport, right_x, right_top, right_width, right_heights[1]);

  return layout;
}

mjrRect PanelBodyRect(const mjrContext& context, const mjrRect& panel_rect) {
  const int title_height = PanelTitleHeight(context);
  return InsetRect(mjrRect{panel_rect.left, panel_rect.bottom, panel_rect.width,
                           std::max(0, panel_rect.height - title_height)},
                   PanelBodyPadX(context), PanelBodyPadY(context));
}

int InspectorRowsPerPage(const ViewerState& viewer, const mjrRect& panel_rect) {
  const mjrRect body_rect = PanelBodyRect(viewer.con, panel_rect);
  if (body_rect.height <= 0) {
    return 1;
  }
  return std::max(1, body_rect.height / PanelLineHeight(viewer.con));
}

[[maybe_unused]] void ClampInspectorPages(const ViewerState& viewer, const WindowUi& ui, int& joint_page,
                                          int& actuator_page) {
  const int joint_rows = InspectorRowsPerPage(viewer, ui.panels[kPanelJoints].rect);
  const int actuator_rows = InspectorRowsPerPage(viewer, ui.panels[kPanelActuators].rect);
  joint_page = std::clamp(joint_page, 0, PageCount(viewer.model->njnt, joint_rows) - 1);
  actuator_page = std::clamp(actuator_page, 0, PageCount(viewer.model->nu, actuator_rows) - 1);
}

void ClampPanelPosition(PanelState& panel, const mjrRect& viewport, const mjrRect& panel_rect) {
  panel.left = std::clamp(panel.left, 0, std::max(0, viewport.width - panel_rect.width));
  panel.top = std::clamp(panel.top, 0, std::max(0, viewport.height - panel_rect.height));
}

WindowUi ComputeWindowUi(ViewerState& viewer, const mjrRect& viewport) {
  WindowUi ui{};
  const bool viewport_changed =
      viewport.width != viewer.last_viewport_width || viewport.height != viewer.last_viewport_height;
  if (viewport_changed && viewer.last_viewport_width > 0 && viewer.last_viewport_height > 0) {
    for (PanelState& state : viewer.panels) {
      if (!state.user_moved) {
        continue;
      }
      state.left = static_cast<int>(
          std::lround(static_cast<double>(state.left) * viewport.width / std::max(1, viewer.last_viewport_width)));
      state.top = static_cast<int>(
          std::lround(static_cast<double>(state.top) * viewport.height / std::max(1, viewer.last_viewport_height)));
    }
  }

  ui.layout = ComputeHudLayout(viewer, viewport, viewer.con);

  const int title_height = PanelTitleHeight(viewer.con);
  const int toggle_width = std::clamp(EstimateTextWidth(viewer.con, "hide") + viewer.con.charHeight, 42, 84);
  const int toggle_height = std::max(16, title_height - 4);

  for (int panel_id = 0; panel_id < kPanelCount; ++panel_id) {
    const mjrRect default_rect = DefaultPanelRect(ui.layout, panel_id);
    PanelState& state = viewer.panels[panel_id];
    if (!state.user_moved) {
      state.left = default_rect.left - viewport.left;
      state.top = RectTop(viewport, default_rect);
    }

    ui.panels[panel_id].rect =
        MakeTopLeftRect(viewport, state.left, state.top, default_rect.width, default_rect.height);
    ClampPanelPosition(state, viewport, ui.panels[panel_id].rect);
    ui.panels[panel_id].rect =
        MakeTopLeftRect(viewport, state.left, state.top, default_rect.width, default_rect.height);
    ui.panels[panel_id].header = mjrRect{ui.panels[panel_id].rect.left,
                                         ui.panels[panel_id].rect.bottom + ui.panels[panel_id].rect.height - title_height,
                                         ui.panels[panel_id].rect.width, title_height};
    ui.panels[panel_id].toggle = mjrRect{ui.panels[panel_id].rect.left + ui.panels[panel_id].rect.width - toggle_width - 4,
                                         ui.panels[panel_id].header.bottom + 2, toggle_width, toggle_height};
  }

  ui.toolbar.rect = ui.layout.toolbar;
  const int button_gap = std::clamp(viewer.con.charHeight / 3, 4, 8);
  const int button_pad = std::clamp(viewer.con.charHeight / 2, 8, 18);
  const int button_height = std::max(18, ui.toolbar.rect.height - 6);
  int button_left = ui.toolbar.rect.left + button_pad;
  const int button_bottom = ui.toolbar.rect.bottom + (ui.toolbar.rect.height - button_height) / 2;
  for (int panel_id = 0; panel_id < kPanelCount; ++panel_id) {
    const int button_width = std::max(48, EstimateTextWidth(viewer.con, kPanelToggleLabels[panel_id]) + button_pad);
    ui.toolbar.buttons[panel_id] = mjrRect{button_left, button_bottom, button_width, button_height};
    button_left += button_width + button_gap;
  }

  viewer.last_viewport_width = viewport.width;
  viewer.last_viewport_height = viewport.height;

  return ui;
}

[[maybe_unused]] WindowUi CurrentWindowUi(GLFWwindow* window) {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return ComputeWindowUi(*g_viewer, mjrRect{0, 0, width, height});
}

[[maybe_unused]] void DrawPanel(const mjrContext& context, const PanelUi& panel_ui, const std::string& title,
                                const std::string& left_body, const std::string& right_body,
                                const PanelStyle& style) {
  const mjrRect& panel_rect = panel_ui.rect;
  if (panel_rect.width <= 0 || panel_rect.height <= 0) {
    return;
  }

  const mjrRect shadow_rect = {panel_rect.left + 3, panel_rect.bottom - 3, panel_rect.width, panel_rect.height};
  mjr_rectangle(shadow_rect, 0.0f, 0.0f, 0.0f, 0.12f);
  mjr_rectangle(panel_rect, style.body_rgba[0], style.body_rgba[1], style.body_rgba[2], style.body_rgba[3]);

  const int title_height = PanelTitleHeight(context);
  const mjrRect title_rect = {panel_rect.left, panel_rect.bottom + panel_rect.height - title_height, panel_rect.width,
                              title_height};
  mjr_label(title_rect, mjFONT_NORMAL, title.c_str(), style.title_rgba[0], style.title_rgba[1], style.title_rgba[2],
            style.title_rgba[3], 0.97f, 0.98f, 0.99f, &context);
  mjr_label(panel_ui.toggle, mjFONT_NORMAL, "hide", 0.10f, 0.16f, 0.20f, 0.95f, 0.92f, 0.95f, 0.98f, &context);

  const mjrRect body_rect = PanelBodyRect(context, panel_rect);
  if (body_rect.width > 0 && body_rect.height > 0) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, body_rect, left_body.c_str(), right_body.c_str(), &context);
  }
}

[[maybe_unused]] void DrawToolbar(const ViewerState& viewer, const ToolbarUi& toolbar) {
  mjr_rectangle(toolbar.rect, 0.02f, 0.05f, 0.08f, 0.42f);

  for (int panel_id = 0; panel_id < kPanelCount; ++panel_id) {
    const bool visible = viewer.panels[panel_id].visible;
    const float r = visible ? 0.12f : 0.07f;
    const float g = visible ? 0.43f : 0.10f;
    const float b = visible ? 0.57f : 0.13f;
    const float a = visible ? 0.95f : 0.55f;
    mjr_label(toolbar.buttons[panel_id], mjFONT_NORMAL, kPanelToggleLabels[panel_id], r, g, b, a, 0.95f, 0.97f, 0.99f,
              &viewer.con);
  }
}

[[maybe_unused]] void BuildStatusPanel(const ViewerState& viewer, const DaruV4MuJoCoController& controller,
                                       int step_count, std::string& left, std::string& right) {
  std::ostringstream lhs;
  std::ostringstream rhs;

  lhs << "State\n"
      << "Time\n"
      << "Step\n"
      << "Mode\n"
      << "Rate\n"
      << "Sel";

  rhs << (viewer.paused ? "PAUSED" : "RUN") << '\n'
      << FormatFixed(viewer.data->time, 2) << " s\n"
      << step_count << '\n'
      << controller.mode() << '\n'
      << FormatFixed(viewer.slow_motion ? 0.25 * viewer.realtime_scale : viewer.realtime_scale, 2) << "x\n"
      << TruncateText(SelectedBodyLabel(viewer), 16);

  left = lhs.str();
  right = rhs.str();
}

[[maybe_unused]] void BuildScenePanel(const ViewerState& viewer, std::string& left, std::string& right) {
  std::ostringstream lhs;
  std::ostringstream rhs;

  lhs << "Camera\n"
      << "Frame\n"
      << "Bodies\n"
      << "Joints\n"
      << "Acts\n"
      << "Viz";

  rhs << CurrentCameraLabel(viewer) << '\n'
      << FrameModeLabel(viewer.opt.frame) << '\n'
      << viewer.model->nbody << '\n'
      << viewer.model->njnt << '\n'
      << viewer.model->nu << '\n'
      << (std::string("C") + FlagChar(viewer.opt.flags[mjVIS_CONTACTPOINT]) + " J" +
          FlagChar(viewer.opt.flags[mjVIS_JOINT]) + " A" + FlagChar(viewer.opt.flags[mjVIS_ACTUATOR]));

  left = lhs.str();
  right = rhs.str();
}

[[maybe_unused]] void BuildControlsPanel(const ViewerState& viewer, std::string& left, std::string& right) {
  std::ostringstream lhs;
  std::ostringstream rhs;

  if (viewer.show_help) {
    lhs << "Pause\n"
        << "Step\n"
        << "Reset\n"
        << "Camera\n"
        << "Visual\n"
        << "Speed\n"
        << "Panels\n"
        << "Drag\n"
        << "Pages\n"
        << "Quit";

    rhs << "SPACE\n"
        << "RIGHT\n"
        << "R\n"
        << "TAB / 0\n"
        << "C J A W S F\n"
        << "- / +\n"
        << "toolbar / hide\n"
        << "header drag\n"
        << "PGUP PGDN HOME END\n"
        << "ESC";
  } else {
    lhs << "Help\n"
        << "Inspect\n"
        << "Panels\n"
        << "Drag";
    rhs << "H\n"
        << "I\n"
        << "toolbar\n"
        << "header";
  }

  left = lhs.str();
  right = rhs.str();
}

std::string MakeInspectorTitle(const std::string& base_title, int start_index, int end_index, int total_count,
                               int page_index, int rows_per_page) {
  std::ostringstream oss;
  oss << base_title << "  ";
  if (total_count == 0) {
    oss << "0 / 0";
  } else {
    oss << (start_index + 1) << "-" << end_index << " / " << total_count << "  (" << (page_index + 1) << "/"
        << PageCount(total_count, rows_per_page) << ")";
  }
  return oss.str();
}

[[maybe_unused]] void BuildJointInspector(const ViewerState& viewer, const mjrRect& panel_rect, std::string& title,
                                          std::string& body) {
  const int total = viewer.model->njnt;
  const int rows_per_page = InspectorRowsPerPage(viewer, panel_rect);
  const int page = std::clamp(viewer.joint_page, 0, PageCount(total, rows_per_page) - 1);
  const int start = page * rows_per_page;
  const int end = std::min(total, start + rows_per_page);
  title = MakeInspectorTitle("Joints", start, end, total, page, rows_per_page);

  std::ostringstream lines;

  if (total == 0) {
    body = "No joints";
    return;
  }

  for (int jid = start; jid < end; ++jid) {
    const int joint_type = viewer.model->jnt_type[jid];
    const int qadr = viewer.model->jnt_qposadr[jid];
    const int dadr = viewer.model->jnt_dofadr[jid];
    const int qcount = JointQposCount(joint_type);
    const int dcount = JointDofCount(joint_type);

    const double q = (qadr >= 0 && qcount > 0) ? viewer.data->qpos[qadr] : 0.0;
    const double dq = (dadr >= 0 && dcount > 0) ? viewer.data->qvel[dadr] : 0.0;
    const double tau =
        (dadr >= 0 && dcount > 0) ? (viewer.data->qfrc_actuator[dadr] + viewer.data->qfrc_applied[dadr]) : 0.0;

    lines << std::setw(2) << jid << "  " << TruncateText(SafeName(viewer.model, mjOBJ_JOINT, jid, "joint"), 16)
          << " | " << JointTypeName(joint_type) << " q" << FormatSigned(q, 2) << " v" << FormatSigned(dq, 2) << " t"
          << FormatSigned(tau, 2);

    if (jid + 1 < end) {
      lines << '\n';
    }
  }

  body = lines.str();
}

[[maybe_unused]] void BuildActuatorInspector(const ViewerState& viewer, const mjrRect& panel_rect, std::string& title,
                                             std::string& body) {
  const int total = viewer.model->nu;
  const int rows_per_page = InspectorRowsPerPage(viewer, panel_rect);
  const int page = std::clamp(viewer.actuator_page, 0, PageCount(total, rows_per_page) - 1);
  const int start = page * rows_per_page;
  const int end = std::min(total, start + rows_per_page);
  title = MakeInspectorTitle("Actuators", start, end, total, page, rows_per_page);

  std::ostringstream lines;

  if (total == 0) {
    body = "No actuators";
    return;
  }

  for (int aid = start; aid < end; ++aid) {
    lines << std::setw(2) << aid << "  " << TruncateText(SafeName(viewer.model, mjOBJ_ACTUATOR, aid, "act"), 16)
          << " | " << TruncateText(ActuatorTargetName(viewer.model, aid), 12) << " u"
          << FormatSigned(viewer.data->ctrl[aid], 2) << " f" << FormatSigned(viewer.data->actuator_force[aid], 2)
          << " " << ActuatorModeName(viewer.model, aid);

    if (aid + 1 < end) {
      lines << '\n';
    }
  }

  body = lines.str();
}

int ApproxCharsForWidth(const mjrContext& context, int width) {
  return std::max(4, width / std::max(5, context.charHeight * 11 / 20));
}

std::string FitTextToWidth(const mjrContext& context, const std::string& text, int width) {
  if (width <= 0) {
    return "";
  }
  const int max_chars = ApproxCharsForWidth(context, width);
  return TruncateText(text, static_cast<std::size_t>(max_chars));
}

bool AnyFavorite(const std::vector<unsigned char>& values) {
  return std::any_of(values.begin(), values.end(), [](unsigned char value) { return value != 0; });
}

bool IsFavorite(const ViewerState& viewer, int category, int item_index) {
  switch (category) {
    case kCategoryStatus:
      return item_index >= 0 && item_index < static_cast<int>(viewer.favorite_status.size()) &&
             viewer.favorite_status[static_cast<std::size_t>(item_index)];
    case kCategoryScene:
      return item_index >= 0 && item_index < static_cast<int>(viewer.favorite_scene.size()) &&
             viewer.favorite_scene[static_cast<std::size_t>(item_index)];
    case kCategoryKeys:
      return item_index >= 0 && item_index < static_cast<int>(viewer.favorite_keys.size()) &&
             viewer.favorite_keys[static_cast<std::size_t>(item_index)];
    case kCategoryJoints:
      return item_index >= 0 && item_index < static_cast<int>(viewer.favorite_joints.size()) &&
             viewer.favorite_joints[static_cast<std::size_t>(item_index)] != 0;
    case kCategoryActuators:
      return item_index >= 0 && item_index < static_cast<int>(viewer.favorite_actuators.size()) &&
             viewer.favorite_actuators[static_cast<std::size_t>(item_index)] != 0;
    default:
      return false;
  }
}

void SetFavorite(ViewerState& viewer, int category, int item_index, bool favorite) {
  switch (category) {
    case kCategoryStatus:
      if (item_index >= 0 && item_index < static_cast<int>(viewer.favorite_status.size())) {
        viewer.favorite_status[static_cast<std::size_t>(item_index)] = favorite;
      }
      break;
    case kCategoryScene:
      if (item_index >= 0 && item_index < static_cast<int>(viewer.favorite_scene.size())) {
        viewer.favorite_scene[static_cast<std::size_t>(item_index)] = favorite;
      }
      break;
    case kCategoryKeys:
      if (item_index >= 0 && item_index < static_cast<int>(viewer.favorite_keys.size())) {
        viewer.favorite_keys[static_cast<std::size_t>(item_index)] = favorite;
      }
      break;
    case kCategoryJoints:
      if (item_index >= 0 && item_index < static_cast<int>(viewer.favorite_joints.size())) {
        viewer.favorite_joints[static_cast<std::size_t>(item_index)] = favorite ? 1 : 0;
      }
      break;
    case kCategoryActuators:
      if (item_index >= 0 && item_index < static_cast<int>(viewer.favorite_actuators.size())) {
        viewer.favorite_actuators[static_cast<std::size_t>(item_index)] = favorite ? 1 : 0;
      }
      break;
    default:
      break;
  }
}

void ToggleFavorite(ViewerState& viewer, int category, int item_index) {
  SetFavorite(viewer, category, item_index, !IsFavorite(viewer, category, item_index));
}

[[maybe_unused]] std::vector<int> FavoriteJointIds(const ViewerState& viewer, int limit = -1) {
  std::vector<int> ids;
  ids.reserve(viewer.favorite_joints.size());
  for (int jid = 0; jid < static_cast<int>(viewer.favorite_joints.size()); ++jid) {
    if (viewer.favorite_joints[static_cast<std::size_t>(jid)] == 0) {
      continue;
    }
    ids.push_back(jid);
    if (limit > 0 && static_cast<int>(ids.size()) >= limit) {
      break;
    }
  }
  return ids;
}

void ClearJointHistory(ViewerState& viewer) {
  for (JointHistory& history : viewer.joint_history) {
    history.q.fill(0.0f);
    history.v.fill(0.0f);
    history.tau.fill(0.0f);
  }
  viewer.joint_history_time.fill(0.0f);
  viewer.joint_history_cursor = 0;
  viewer.joint_history_count = 0;
  viewer.last_history_time = -1.0;
}

void UpdateJointHistory(ViewerState& viewer) {
  if (!viewer.model || !viewer.data || viewer.model->njnt <= 0 || viewer.joint_history.empty()) {
    return;
  }
  if (viewer.last_history_time == viewer.data->time) {
    return;
  }

  const int sample = viewer.joint_history_cursor;
  viewer.joint_history_time[static_cast<std::size_t>(sample)] = static_cast<float>(viewer.data->time);
  for (int jid = 0; jid < viewer.model->njnt; ++jid) {
    const int joint_type = viewer.model->jnt_type[jid];
    const int qadr = viewer.model->jnt_qposadr[jid];
    const int dadr = viewer.model->jnt_dofadr[jid];
    const int qcount = JointQposCount(joint_type);
    const int dcount = JointDofCount(joint_type);
    const float q = (qadr >= 0 && qcount > 0) ? static_cast<float>(viewer.data->qpos[qadr]) : 0.0f;
    const float dq = (dadr >= 0 && dcount > 0) ? static_cast<float>(viewer.data->qvel[dadr]) : 0.0f;
    const float tau =
        (dadr >= 0 && dcount > 0) ? static_cast<float>(viewer.data->qfrc_actuator[dadr] + viewer.data->qfrc_applied[dadr]) : 0.0f;

    viewer.joint_history[static_cast<std::size_t>(jid)].q[static_cast<std::size_t>(sample)] = q;
    viewer.joint_history[static_cast<std::size_t>(jid)].v[static_cast<std::size_t>(sample)] = dq;
    viewer.joint_history[static_cast<std::size_t>(jid)].tau[static_cast<std::size_t>(sample)] = tau;
  }

  viewer.joint_history_cursor = (viewer.joint_history_cursor + 1) % kJointHistorySamples;
  viewer.joint_history_count = std::min(kJointHistorySamples, viewer.joint_history_count + 1);
  viewer.last_history_time = viewer.data->time;
}

float JointHistoryValue(const ViewerState& viewer, int jid, int metric, int ordered_index) {
  if (jid < 0 || jid >= static_cast<int>(viewer.joint_history.size()) || ordered_index < 0 ||
      ordered_index >= viewer.joint_history_count) {
    return 0.0f;
  }

  const int start = (viewer.joint_history_count < kJointHistorySamples) ? 0 : viewer.joint_history_cursor;
  const int sample = (start + ordered_index) % kJointHistorySamples;
  const JointHistory& history = viewer.joint_history[static_cast<std::size_t>(jid)];
  switch (metric) {
    case 0:
      return history.q[static_cast<std::size_t>(sample)];
    case 1:
      return history.v[static_cast<std::size_t>(sample)];
    case 2:
      return history.tau[static_cast<std::size_t>(sample)];
    default:
      return 0.0f;
  }
}

float JointHistoryTimeValue(const ViewerState& viewer, int ordered_index) {
  if (ordered_index < 0 || ordered_index >= viewer.joint_history_count) {
    return 0.0f;
  }
  const int start = (viewer.joint_history_count < kJointHistorySamples) ? 0 : viewer.joint_history_cursor;
  const int sample = (start + ordered_index) % kJointHistorySamples;
  return viewer.joint_history_time[static_cast<std::size_t>(sample)];
}

std::vector<DrawerItem> BuildStatusItems(const ViewerState& viewer) {
  std::vector<DrawerItem> items;
  items.reserve(6);

  const std::array<const char*, 6> labels = {"State", "Time", "Step", "Mode", "Rate", "Selected"};
  const std::array<std::string, 6> values = {
      viewer.paused ? "PAUSED" : "RUN",
      FormatFixed(viewer.data->time, 2) + " s",
      std::to_string(viewer.step_count),
      viewer.controller ? std::to_string(viewer.controller->mode()) : "?",
      FormatFixed(viewer.slow_motion ? 0.25 * viewer.realtime_scale : viewer.realtime_scale, 2) + "x",
      SelectedBodyLabel(viewer),
  };

  for (std::size_t i = 0; i < labels.size(); ++i) {
    DrawerItem item;
    item.source_category = kCategoryStatus;
    item.item_index = static_cast<int>(i);
    item.can_favorite = true;
    item.favorite = viewer.favorite_status[i];
    item.label = labels[i];
    item.value = values[i];
    item.columns[0] = item.label;
    item.columns[1] = item.value;
    item.column_count = 2;
    items.push_back(std::move(item));
  }
  return items;
}

std::vector<DrawerItem> BuildSceneItems(const ViewerState& viewer) {
  std::vector<DrawerItem> items;
  items.reserve(6);

  const std::array<const char*, 6> labels = {"Camera", "Frame", "Bodies", "Joints", "Acts", "Viz"};
  const std::array<std::string, 6> values = {
      CurrentCameraLabel(viewer),
      FrameModeLabel(viewer.opt.frame),
      std::to_string(viewer.model->nbody),
      std::to_string(viewer.model->njnt),
      std::to_string(viewer.model->nu),
      std::string("C") + FlagChar(viewer.opt.flags[mjVIS_CONTACTPOINT]) + " J" +
          FlagChar(viewer.opt.flags[mjVIS_JOINT]) + " A" + FlagChar(viewer.opt.flags[mjVIS_ACTUATOR]),
  };

  for (std::size_t i = 0; i < labels.size(); ++i) {
    DrawerItem item;
    item.source_category = kCategoryScene;
    item.item_index = static_cast<int>(i);
    item.can_favorite = true;
    item.favorite = viewer.favorite_scene[i];
    item.label = labels[i];
    item.value = values[i];
    item.columns[0] = item.label;
    item.columns[1] = item.value;
    item.column_count = 2;
    items.push_back(std::move(item));
  }
  return items;
}

std::vector<DrawerItem> BuildKeyItems(const ViewerState& viewer, bool include_hidden) {
  std::vector<DrawerItem> items;
  items.reserve(kKeyRows.size());

  for (std::size_t i = 0; i < kKeyRows.size(); ++i) {
    if (!include_hidden && !viewer.show_help && !kKeyRows[i].compact_visible) {
      continue;
    }
    DrawerItem item;
    item.source_category = kCategoryKeys;
    item.item_index = static_cast<int>(i);
    item.can_favorite = true;
    item.favorite = viewer.favorite_keys[i];
    item.label = kKeyRows[i].label;
    item.value = kKeyRows[i].value;
    item.columns[0] = item.label;
    item.columns[1] = item.value;
    item.column_count = 2;
    items.push_back(std::move(item));
  }
  return items;
}

std::vector<DrawerItem> BuildJointItems(const ViewerState& viewer) {
  std::vector<DrawerItem> items;
  items.reserve(static_cast<std::size_t>(viewer.model->njnt));

  for (int jid = 0; jid < viewer.model->njnt; ++jid) {
    const int joint_type = viewer.model->jnt_type[jid];
    const int qadr = viewer.model->jnt_qposadr[jid];
    const int dadr = viewer.model->jnt_dofadr[jid];
    const int qcount = JointQposCount(joint_type);
    const int dcount = JointDofCount(joint_type);
    const double q = (qadr >= 0 && qcount > 0) ? viewer.data->qpos[qadr] : 0.0;
    const double dq = (dadr >= 0 && dcount > 0) ? viewer.data->qvel[dadr] : 0.0;
    const double tau =
        (dadr >= 0 && dcount > 0) ? (viewer.data->qfrc_actuator[dadr] + viewer.data->qfrc_applied[dadr]) : 0.0;

    std::ostringstream label;
    label << jid << "  " << SafeName(viewer.model, mjOBJ_JOINT, jid, "joint");
    std::ostringstream value;
    value << JointTypeShortName(joint_type) << " q" << FormatSigned(q, 1) << " v" << FormatSigned(dq, 1) << " t"
          << FormatSigned(tau, 1);

    DrawerItem item;
    item.source_category = kCategoryJoints;
    item.item_index = jid;
    item.can_favorite = true;
    item.favorite = IsFavorite(viewer, kCategoryJoints, jid);
    item.label = label.str();
    item.value = value.str();
    item.columns[0] = std::to_string(jid);
    item.columns[1] = SafeName(viewer.model, mjOBJ_JOINT, jid, "joint");
    item.columns[2] = JointTypeShortName(joint_type);
    item.columns[3] = FormatSigned(q, 1);
    item.columns[4] = FormatSigned(dq, 1);
    item.columns[5] = FormatSigned(tau, 1);
    item.column_count = 6;
    items.push_back(std::move(item));
  }

  return items;
}

std::vector<DrawerItem> BuildActuatorItems(const ViewerState& viewer) {
  std::vector<DrawerItem> items;
  items.reserve(static_cast<std::size_t>(viewer.model->nu));

  for (int aid = 0; aid < viewer.model->nu; ++aid) {
    std::ostringstream label;
    label << aid << "  " << SafeName(viewer.model, mjOBJ_ACTUATOR, aid, "act");
    std::ostringstream value;
    value << ActuatorTargetName(viewer.model, aid) << " u" << FormatSigned(viewer.data->ctrl[aid], 1) << " f"
          << FormatSigned(viewer.data->actuator_force[aid], 1) << " " << ActuatorModeName(viewer.model, aid);

    DrawerItem item;
    item.source_category = kCategoryActuators;
    item.item_index = aid;
    item.can_favorite = true;
    item.favorite = IsFavorite(viewer, kCategoryActuators, aid);
    item.label = label.str();
    item.value = value.str();
    item.columns[0] = std::to_string(aid);
    item.columns[1] = SafeName(viewer.model, mjOBJ_ACTUATOR, aid, "act");
    item.columns[2] = ActuatorTargetName(viewer.model, aid);
    item.columns[3] = FormatSigned(viewer.data->ctrl[aid], 1);
    item.columns[4] = FormatSigned(viewer.data->actuator_force[aid], 1);
    item.columns[5] = ActuatorModeName(viewer.model, aid);
    item.column_count = 6;
    items.push_back(std::move(item));
  }

  return items;
}

std::string FavoriteSectionName(int category) {
  switch (category) {
    case kCategoryStatus:
      return "Status";
    case kCategoryScene:
      return "Scene";
    case kCategoryKeys:
      return "Keys";
    case kCategoryJoints:
      return "Joint";
    case kCategoryActuators:
      return "Actuator";
    default:
      return "Item";
  }
}

std::vector<DrawerItem> BuildFavoriteItems(const ViewerState& viewer) {
  std::vector<DrawerItem> items;

  const auto append_favorites = [&](std::vector<DrawerItem> source_items) {
    for (DrawerItem& item : source_items) {
      if (!item.favorite) {
        continue;
      }
      const std::string group = FavoriteSectionName(item.source_category);
      item.label = group + " / " + item.label;
      item.columns[0] = group;
      item.columns[1] = item.label;
      item.columns[2] = item.value;
      item.column_count = 3;
      items.push_back(std::move(item));
    }
  };

  if (std::any_of(viewer.favorite_status.begin(), viewer.favorite_status.end(), [](bool value) { return value; })) {
    append_favorites(BuildStatusItems(viewer));
  }
  if (std::any_of(viewer.favorite_scene.begin(), viewer.favorite_scene.end(), [](bool value) { return value; })) {
    append_favorites(BuildSceneItems(viewer));
  }
  if (std::any_of(viewer.favorite_keys.begin(), viewer.favorite_keys.end(), [](bool value) { return value; })) {
    append_favorites(BuildKeyItems(viewer, true));
  }
  if (AnyFavorite(viewer.favorite_joints)) {
    append_favorites(BuildJointItems(viewer));
  }
  if (AnyFavorite(viewer.favorite_actuators)) {
    append_favorites(BuildActuatorItems(viewer));
  }

  if (items.empty()) {
    DrawerItem item;
    item.source_category = kCategoryFavorites;
    item.item_index = -1;
    item.label = "No favorites yet";
    item.value = "click [ ] on any row";
    item.columns[0] = "Info";
    item.columns[1] = item.label;
    item.columns[2] = item.value;
    item.column_count = 3;
    items.push_back(std::move(item));
  }

  return items;
}

std::vector<DrawerItem> BuildCategoryItems(const ViewerState& viewer, int category) {
  std::vector<DrawerItem> items;
  switch (category) {
    case kCategoryFavorites:
      return BuildFavoriteItems(viewer);
    case kCategoryStatus:
      items = BuildStatusItems(viewer);
      break;
    case kCategoryScene:
      items = BuildSceneItems(viewer);
      break;
    case kCategoryKeys:
      items = BuildKeyItems(viewer, false);
      break;
    case kCategoryJoints:
      items = BuildJointItems(viewer);
      break;
    case kCategoryActuators:
      items = BuildActuatorItems(viewer);
      break;
    default:
      return {};
  }

  if (viewer.favorites_only[category]) {
    items.erase(std::remove_if(items.begin(), items.end(), [](const DrawerItem& item) { return !item.favorite; }), items.end());
    if (items.empty()) {
      DrawerItem placeholder;
      placeholder.source_category = category;
      placeholder.item_index = -1;
      placeholder.label = "No favorites in this view";
      placeholder.value = "toggle FAV to show all";
      if (category == kCategoryJoints || category == kCategoryActuators) {
        placeholder.columns[0] = "-";
        placeholder.columns[1] = placeholder.label;
        placeholder.columns[2] = placeholder.value;
        placeholder.columns[3] = "";
        placeholder.columns[4] = "";
        placeholder.columns[5] = "";
        placeholder.column_count = 6;
      } else {
        placeholder.columns[0] = "INFO";
        placeholder.columns[1] = placeholder.value;
        placeholder.column_count = 2;
      }
      items.push_back(std::move(placeholder));
    }
  }

  return items;
}

struct TableSpec {
  std::array<const char*, 6> headers{};
  std::array<int, 6> weights{};
  int column_count = 0;
};

TableSpec GetTableSpec(int category) {
  switch (category) {
    case kCategoryFavorites:
      return {{{"GROUP", "ITEM", "VALUE", "", "", ""}}, {{18, 34, 48, 0, 0, 0}}, 3};
    case kCategoryStatus:
    case kCategoryScene:
    case kCategoryKeys:
      return {{{"FIELD", "VALUE", "", "", "", ""}}, {{38, 62, 0, 0, 0, 0}}, 2};
    case kCategoryJoints:
      return {{{"ID", "NAME", "T", "Q", "V", "TAU"}}, {{8, 40, 8, 14, 14, 16}}, 6};
    case kCategoryActuators:
      return {{{"ID", "NAME", "TARGET", "U", "F", "MODE"}}, {{6, 31, 24, 11, 12, 16}}, 6};
    default:
      return {};
  }
}

const char* ActionButtonText(const ViewerState& viewer, int action_id) {
  switch (action_id) {
    case kActionPause:
      return viewer.paused ? "RUN" : "PAUSE";
    case kActionStep:
      return "STEP";
    case kActionReset:
      return "RESET";
    case kActionFavoritesOnly:
      return viewer.favorites_only[viewer.active_category] ? "FAV*" : "FAV";
    default:
      return "";
  }
}

bool CategoryCanFilterFavorites(int category) { return category != kCategoryFavorites; }

std::array<mjrRect, 6> ComputeColumnRects(const mjrRect& row_rect, int favorite_width, const TableSpec& spec) {
  std::array<mjrRect, 6> rects{};
  const mjrRect inner = InsetRect(row_rect, 6, 3);
  const int cell_gap = 4;
  const int text_left = inner.left + favorite_width + cell_gap;
  const int text_width = std::max(0, inner.width - favorite_width - cell_gap * std::max(0, spec.column_count - 1));

  int total_weight = 0;
  for (int i = 0; i < spec.column_count; ++i) {
    total_weight += std::max(1, spec.weights[i]);
  }

  int used_width = 0;
  int left = text_left;
  for (int i = 0; i < spec.column_count; ++i) {
    const int width =
        (i + 1 == spec.column_count)
            ? std::max(0, text_width - used_width)
            : std::max(0, text_width * std::max(1, spec.weights[i]) / std::max(1, total_weight));
    rects[i] = mjrRect{left, inner.bottom, width, inner.height};
    left += width + cell_gap;
    used_width += width;
  }
  return rects;
}

int DockWidth(const mjrContext& context) { return std::clamp(context.charHeight * 2 + 16, 50, 68); }

int WindowHeaderHeight(const mjrContext& context) { return std::clamp(context.charHeight + 14, 24, 36); }

int WindowColumnsHeight(const mjrContext& context) { return std::clamp(context.charHeight + 8, 20, 28); }

int WindowFooterHeight(const mjrContext& context) { return std::clamp(context.charHeight + 10, 22, 32); }

int WindowRowHeight(const mjrContext& context) { return std::clamp(context.charHeight + 8, 20, 30); }

int GraphControlButtonWidth(const mjrContext& context) {
  return std::clamp(EstimateTextWidth(context, "AUTO") + context.charHeight / 3, 34, 58);
}

int GraphValueButtonWidth(const mjrContext& context) {
  return std::clamp(EstimateTextWidth(context, "X 100.0s") + context.charHeight / 3, 64, 90);
}

int GraphControlsHeight(const mjrContext& context) {
  return std::clamp(WindowFooterHeight(context) * 2 + 6, 42, 64);
}

int WindowRowGap(const mjrContext& context) { return std::clamp(context.charHeight / 8, 1, 4); }

int WindowBodyPadX(const mjrContext& context) { return std::clamp(context.charHeight / 4, 4, 10); }

int WindowBodyPadY(const mjrContext& context) { return std::clamp(context.charHeight / 5, 3, 8); }

void BringWindowToFront(ViewerState& viewer, int category) {
  auto it = std::find(viewer.window_order.begin(), viewer.window_order.end(), category);
  if (it == viewer.window_order.end()) {
    return;
  }
  const std::size_t index = static_cast<std::size_t>(std::distance(viewer.window_order.begin(), it));
  for (std::size_t i = index; i + 1 < viewer.window_order.size(); ++i) {
    viewer.window_order[i] = viewer.window_order[i + 1];
  }
  viewer.window_order.back() = category;
}

int TopVisibleWindowCategory(const ViewerState& viewer) {
  for (auto it = viewer.window_order.rbegin(); it != viewer.window_order.rend(); ++it) {
    const int category = *it;
    if (viewer.floating_windows[category].visible) {
      return category;
    }
  }
  return kCategoryStatus;
}

void FocusWindow(ViewerState& viewer, int category) {
  viewer.focused_graph_window = -1;
  viewer.active_category = category;
  BringWindowToFront(viewer, category);
}

void OpenWindow(ViewerState& viewer, int category) {
  viewer.floating_windows[category].visible = true;
  viewer.floating_windows[category].minimized = false;
  FocusWindow(viewer, category);
}

void CloseWindow(ViewerState& viewer, int category) {
  viewer.floating_windows[category].visible = false;
  viewer.floating_windows[category].minimized = false;
  if (viewer.active_category == category) {
    viewer.active_category = TopVisibleWindowCategory(viewer);
  }
}

int DefaultGraphJoint(const ViewerState& viewer) {
  for (int jid = 0; jid < static_cast<int>(viewer.favorite_joints.size()); ++jid) {
    if (viewer.favorite_joints[static_cast<std::size_t>(jid)] != 0) {
      return jid;
    }
  }
  return 0;
}

void BringGraphWindowToFront(ViewerState& viewer, int graph_index) {
  auto it = std::find(viewer.graph_window_order.begin(), viewer.graph_window_order.end(), graph_index);
  if (it == viewer.graph_window_order.end()) {
    return;
  }
  const std::size_t index = static_cast<std::size_t>(std::distance(viewer.graph_window_order.begin(), it));
  for (std::size_t i = index; i + 1 < viewer.graph_window_order.size(); ++i) {
    viewer.graph_window_order[i] = viewer.graph_window_order[i + 1];
  }
  viewer.graph_window_order.back() = graph_index;
}

void FocusGraphWindow(ViewerState& viewer, int graph_index) {
  viewer.focused_graph_window = graph_index;
  BringGraphWindowToFront(viewer, graph_index);
}

void CancelGraphEdit(ViewerState& viewer) {
  viewer.editing_graph_window = -1;
  viewer.editing_graph_field = kGraphEditNone;
  viewer.graph_edit_buffer.clear();
}

void BeginGraphEdit(ViewerState& viewer, int graph_index, int field) {
  if (graph_index < 0 || graph_index >= static_cast<int>(viewer.graph_windows.size())) {
    CancelGraphEdit(viewer);
    return;
  }
  viewer.editing_graph_window = graph_index;
  viewer.editing_graph_field = field;
  const GraphWindowState& state = viewer.graph_windows[static_cast<std::size_t>(graph_index)];
  if (field == kGraphEditXSeconds) {
    viewer.graph_edit_buffer = FormatFixed(state.x_seconds, 1);
  } else if (field == kGraphEditYLimit) {
    viewer.graph_edit_buffer = FormatFixed(state.y_manual_limit, 1);
  } else {
    viewer.graph_edit_buffer.clear();
  }
}

void CommitGraphEdit(ViewerState& viewer) {
  if (viewer.editing_graph_window < 0 || viewer.editing_graph_window >= static_cast<int>(viewer.graph_windows.size()) ||
      viewer.editing_graph_field == kGraphEditNone) {
    CancelGraphEdit(viewer);
    return;
  }

  char* end = nullptr;
  const double value = std::strtod(viewer.graph_edit_buffer.c_str(), &end);
  if (end != viewer.graph_edit_buffer.c_str() && end && *end == '\0') {
    GraphWindowState& state = viewer.graph_windows[static_cast<std::size_t>(viewer.editing_graph_window)];
    if (viewer.editing_graph_field == kGraphEditXSeconds) {
      state.x_seconds = std::clamp(value, 0.1, 300.0);
    } else if (viewer.editing_graph_field == kGraphEditYLimit) {
      state.y_auto = false;
      state.y_manual_limit = std::clamp(value, 0.05, 100000.0);
    }
  }
  CancelGraphEdit(viewer);
}

bool HandleGraphEditKey(ViewerState& viewer, int key) {
  if (viewer.editing_graph_field == kGraphEditNone) {
    return false;
  }

  switch (key) {
    case GLFW_KEY_ESCAPE:
      CancelGraphEdit(viewer);
      return true;
    case GLFW_KEY_ENTER:
    case GLFW_KEY_KP_ENTER:
      CommitGraphEdit(viewer);
      return true;
    case GLFW_KEY_BACKSPACE:
      if (!viewer.graph_edit_buffer.empty()) {
        viewer.graph_edit_buffer.pop_back();
      }
      return true;
    case GLFW_KEY_PERIOD:
    case GLFW_KEY_KP_DECIMAL:
      if (viewer.graph_edit_buffer.find('.') == std::string::npos) {
        if (viewer.graph_edit_buffer.empty()) {
          viewer.graph_edit_buffer = "0.";
        } else {
          viewer.graph_edit_buffer.push_back('.');
        }
      }
      return true;
    default:
      break;
  }

  if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
    viewer.graph_edit_buffer.push_back(static_cast<char>('0' + (key - GLFW_KEY_0)));
    return true;
  }
  if (key >= GLFW_KEY_KP_0 && key <= GLFW_KEY_KP_9) {
    viewer.graph_edit_buffer.push_back(static_cast<char>('0' + (key - GLFW_KEY_KP_0)));
    return true;
  }

  return false;
}

int OpenGraphWindow(ViewerState& viewer, int preferred_joint = -1) {
  GraphWindowState state;
  state.serial = viewer.next_graph_window_serial++;
  const int last_joint = std::max(0, viewer.model ? viewer.model->njnt - 1 : 0);
  state.selected_joint = std::clamp(preferred_joint >= 0 ? preferred_joint : DefaultGraphJoint(viewer), 0, last_joint);
  state.visible = true;
  state.minimized = false;
  state.user_moved = false;
  viewer.graph_windows.push_back(state);
  const int graph_index = static_cast<int>(viewer.graph_windows.size()) - 1;
  viewer.graph_window_order.push_back(graph_index);
  FocusGraphWindow(viewer, graph_index);
  return graph_index;
}

void CloseGraphWindow(ViewerState& viewer, int graph_index) {
  if (graph_index < 0 || graph_index >= static_cast<int>(viewer.graph_windows.size())) {
    return;
  }
  viewer.graph_windows[static_cast<std::size_t>(graph_index)].visible = false;
  viewer.graph_windows[static_cast<std::size_t>(graph_index)].minimized = false;
  if (viewer.focused_graph_window == graph_index) {
    viewer.focused_graph_window = -1;
  }
  if (viewer.editing_graph_window == graph_index) {
    CancelGraphEdit(viewer);
  }
}

template <std::size_t N>
double StepPresetValue(double value, const std::array<double, N>& presets, int direction) {
  static_assert(N > 0, "presets must not be empty");
  int best = 0;
  double best_dist = std::abs(presets[0] - value);
  for (int i = 1; i < static_cast<int>(N); ++i) {
    const double dist = std::abs(presets[static_cast<std::size_t>(i)] - value);
    if (dist < best_dist) {
      best = i;
      best_dist = dist;
    }
  }
  best = std::clamp(best + direction, 0, static_cast<int>(N) - 1);
  return presets[static_cast<std::size_t>(best)];
}

int WindowDefaultWidth(const ViewerState& viewer, int category, const mjrRect& viewport) {
  (void)viewer;
  switch (category) {
    case kCategoryFavorites:
      return std::clamp(viewport.width * 31 / 100, 380, 560);
    case kCategoryStatus:
    case kCategoryScene:
    case kCategoryKeys:
      return std::clamp(viewport.width * 19 / 100, 250, 340);
    case kCategoryJoints:
      return std::clamp(viewport.width * 28 / 100, 390, 540);
    case kCategoryActuators:
      return std::clamp(viewport.width * 33 / 100, 450, 680);
    default:
      return std::clamp(viewport.width * 24 / 100, 300, 420);
  }
}

int WindowDefaultRows(const ViewerState& viewer, int category, const mjrRect& viewport) {
  const int tall_rows = std::clamp((viewport.height - 210) / std::max(22, WindowRowHeight(viewer.con)), 5, 16);
  switch (category) {
    case kCategoryStatus:
    case kCategoryScene:
      return 5;
    case kCategoryKeys:
      return viewer.show_help ? 9 : 4;
    case kCategoryFavorites:
      return std::clamp(tall_rows - 2, 5, 12);
    case kCategoryJoints:
      return std::clamp(tall_rows, 7, 15);
    case kCategoryActuators:
      return std::clamp(tall_rows - 1, 7, 14);
    default:
      return 7;
  }
}

std::pair<int, int> DefaultWindowTopLeft(const ViewerState& viewer, int category, const mjrRect& viewport,
                                         const DockUi& dock, int width) {
  (void)viewer;
  const int pad = std::clamp(std::min(viewport.width, viewport.height) / 140, 6, 12);
  const int left_anchor = dock.rect.left + dock.rect.width + pad;
  const int right_anchor = std::max(left_anchor, viewport.width - width - pad);

  switch (category) {
    case kCategoryFavorites:
      return {left_anchor + 20, pad + 20};
    case kCategoryStatus:
      return {left_anchor + 6, pad + 16};
    case kCategoryScene:
      return {left_anchor + 28, pad + 54};
    case kCategoryKeys:
      return {left_anchor + 50, pad + 92};
    case kCategoryJoints:
      return {std::max(left_anchor, right_anchor - 36), pad + 18};
    case kCategoryActuators:
      return {right_anchor, pad + 64};
    default:
      return {left_anchor, pad + 20};
  }
}

void ClampFloatingWindow(FloatingWindowState& state, const mjrRect& viewport, int width, int height) {
  state.left = std::clamp(state.left, 0, std::max(0, viewport.width - width));
  state.top = std::clamp(state.top, 0, std::max(0, viewport.height - height));
}

FloatingWindowUi ComputeFloatingWindowUi(ViewerState& viewer, int category, const mjrRect& viewport, const DockUi& dock) {
  FloatingWindowUi ui{};
  ui.category = category;
  const FloatingWindowState& state = viewer.floating_windows[category];
  ui.visible = state.visible;
  ui.minimized = state.minimized;
  if (!ui.visible) {
    return ui;
  }

  const std::vector<DrawerItem> items = BuildCategoryItems(viewer, category);
  ui.total_items = static_cast<int>(items.size());
  const int row_height = WindowRowHeight(viewer.con);
  const int row_gap = WindowRowGap(viewer.con);
  const int header_height = WindowHeaderHeight(viewer.con);
  const int columns_height = WindowColumnsHeight(viewer.con);
  const int footer_height = WindowFooterHeight(viewer.con);
  const int body_pad_y = WindowBodyPadY(viewer.con);
  const int default_width = WindowDefaultWidth(viewer, category, viewport);
  const int preferred_rows = WindowDefaultRows(viewer, category, viewport);
  const int rows_per_page = std::max(1, preferred_rows);
  ui.total_pages = PageCount(ui.total_items, rows_per_page);

  int& page_index = viewer.category_pages[category];
  page_index = std::clamp(page_index, 0, ui.total_pages - 1);
  ui.page_index = page_index;
  const int start = page_index * rows_per_page;
  const int end = std::min(ui.total_items, start + rows_per_page);
  ui.range_start = ui.total_items == 0 ? 0 : start + 1;
  ui.range_end = end;

  const int visible_rows = std::max(1, end - start);
  const int body_height = 2 * body_pad_y + visible_rows * row_height + std::max(0, visible_rows - 1) * row_gap;
  const int full_height = header_height + columns_height + body_height + footer_height;
  const int current_height = ui.minimized ? header_height : full_height;

  FloatingWindowState& mutable_state = viewer.floating_windows[category];
  if (!mutable_state.user_moved) {
    const auto [left, top] = DefaultWindowTopLeft(viewer, category, viewport, dock, default_width);
    mutable_state.left = left;
    mutable_state.top = top;
  }
  ClampFloatingWindow(mutable_state, viewport, default_width, current_height);
  ui.rect = MakeTopLeftRect(viewport, mutable_state.left, mutable_state.top, default_width, current_height);
  ui.header_rect = mjrRect{ui.rect.left, ui.rect.bottom + ui.rect.height - header_height, ui.rect.width, header_height};

  const int button_size = std::max(14, header_height - 10);
  const int button_bottom = ui.header_rect.bottom + (ui.header_rect.height - button_size) / 2;
  const int close_left = ui.header_rect.left + ui.header_rect.width - button_size - 6;
  const int minimize_left = close_left - button_size - 4;
  ui.close_button = mjrRect{close_left, button_bottom, button_size, button_size};
  ui.minimize_button = mjrRect{minimize_left, button_bottom, button_size, button_size};

  const int icon_size = std::max(12, header_height - 12);
  ui.icon_rect = mjrRect{ui.header_rect.left + 8, ui.header_rect.bottom + (ui.header_rect.height - icon_size) / 2, icon_size,
                         icon_size};
  const int title_left = ui.icon_rect.left + ui.icon_rect.width + 8;
  const int title_width = std::max(0, minimize_left - title_left - 8);
  ui.title_rect = mjrRect{title_left, ui.header_rect.bottom + ui.header_rect.height / 2, title_width,
                          std::max(0, ui.header_rect.height / 2 - 2)};
  ui.subtitle_rect = mjrRect{title_left, ui.header_rect.bottom + 2, title_width, std::max(0, ui.header_rect.height / 2 - 4)};

  if (ui.minimized) {
    return ui;
  }

  ui.columns_rect = mjrRect{ui.rect.left, ui.rect.bottom + footer_height + body_height, ui.rect.width, columns_height};
  ui.body_rect = mjrRect{ui.rect.left, ui.rect.bottom + footer_height, ui.rect.width, body_height};
  ui.footer_rect = mjrRect{ui.rect.left, ui.rect.bottom, ui.rect.width, footer_height};

  const mjrRect rows_rect = InsetRect(ui.body_rect, WindowBodyPadX(viewer.con), WindowBodyPadY(viewer.con));
  int row_bottom = rows_rect.bottom + rows_rect.height - row_height;
  const int favorite_width = std::clamp(EstimateTextWidth(viewer.con, "*") + viewer.con.charHeight / 2, 18, 30);
  for (int item_index = start; item_index < end; ++item_index) {
    DrawerEntryUi entry;
    entry.item = items[static_cast<std::size_t>(item_index)];
    entry.rect = mjrRect{rows_rect.left, row_bottom, rows_rect.width, row_height};
    entry.favorite_button = mjrRect{entry.rect.left + 6, entry.rect.bottom + 4, favorite_width, std::max(0, row_height - 8)};
    ui.entries.push_back(std::move(entry));
    row_bottom -= row_height + row_gap;
  }

  const int footer_button_height = std::max(16, ui.footer_rect.height - 8);
  const int footer_button_width =
      std::clamp(EstimateTextWidth(viewer.con, "RESET") + viewer.con.charHeight / 2, 44, 72);
  const int footer_bottom = ui.footer_rect.bottom + (ui.footer_rect.height - footer_button_height) / 2;
  ui.show_status_actions = category == kCategoryStatus;
  if (ui.show_status_actions) {
    int right = ui.footer_rect.left + ui.footer_rect.width - 8;
    ui.reset_button = mjrRect{right - footer_button_width, footer_bottom, footer_button_width, footer_button_height};
    right -= footer_button_width + 6;
    ui.task_reset_button = mjrRect{right - footer_button_width, footer_bottom, footer_button_width, footer_button_height};
    right -= footer_button_width + 6;
    ui.step_button = mjrRect{right - footer_button_width, footer_bottom, footer_button_width, footer_button_height};
    right -= footer_button_width + 6;
    ui.pause_button = mjrRect{right - footer_button_width, footer_bottom, footer_button_width, footer_button_height};
  }

  ui.show_favorites_button = CategoryCanFilterFavorites(category) && !ui.show_status_actions;
  if (ui.show_favorites_button) {
    ui.favorite_filter_button =
        mjrRect{ui.footer_rect.left + ui.footer_rect.width - footer_button_width - 8, footer_bottom, footer_button_width,
                footer_button_height};
  }
  ui.show_graph_button = category == kCategoryJoints;
  if (ui.show_graph_button) {
    const int right_anchor =
        ui.show_favorites_button ? (ui.favorite_filter_button.left - 6) : (ui.footer_rect.left + ui.footer_rect.width - 8);
    ui.graph_button = mjrRect{right_anchor - footer_button_width, footer_bottom, footer_button_width, footer_button_height};
  }

  ui.show_page_controls = ui.total_pages > 1;
  if (ui.show_page_controls) {
    const int nav_width = std::clamp(EstimateTextWidth(viewer.con, "NEXT") + viewer.con.charHeight / 3, 38, 62);
    ui.prev_button = mjrRect{ui.footer_rect.left + 8, footer_bottom, nav_width, footer_button_height};
    ui.next_button = mjrRect{ui.footer_rect.left + 8 + nav_width + 6, footer_bottom, nav_width, footer_button_height};
  }

  return ui;
}

int GraphWindowDefaultWidth(const ViewerState& viewer, const mjrRect& viewport) {
  const int button_width = GraphControlButtonWidth(viewer.con);
  const int value_width = GraphValueButtonWidth(viewer.con);
  const int control_min_width = 24 + 6 * button_width + 2 * value_width;
  const int body_min_width = 160 + 8 + std::max(220, viewer.con.charHeight * 15);
  const int min_width = std::max({460, control_min_width, body_min_width});
  const int max_width = std::max(min_width, std::min(720, viewport.width - 24));
  return std::clamp(viewport.width * 36 / 100, min_width, max_width);
}

int GraphWindowDefaultHeight(const ViewerState& viewer, const mjrRect& viewport) {
  const int selector_body_height = 8 * WindowRowHeight(viewer.con) + 7 * WindowRowGap(viewer.con) + 12;
  const int selector_total_height =
      WindowColumnsHeight(viewer.con) + WindowFooterHeight(viewer.con) + selector_body_height;
  const int graph_height = std::max(78, viewer.con.charHeight * 4 + 8);
  const int charts_total_height = 3 * graph_height + 2 * 8 + 12;
  const int min_height =
      WindowHeaderHeight(viewer.con) + GraphControlsHeight(viewer.con) + std::max(selector_total_height, charts_total_height);
  const int max_height = std::max(min_height, std::min(560, viewport.height - 24));
  return std::clamp(viewport.height * 46 / 100, min_height, max_height);
}

std::pair<int, int> DefaultGraphWindowTopLeft(const ViewerState& viewer, int graph_index, const mjrRect& viewport,
                                              const DockUi& dock, int width, int height) {
  (void)viewer;
  const int pad = std::clamp(std::min(viewport.width, viewport.height) / 140, 6, 12);
  const int cascade_x = 20 * (graph_index % 5);
  const int cascade_y = 26 * (graph_index % 6);
  const int left = std::max(dock.rect.left + dock.rect.width + pad, viewport.width - width - pad - cascade_x);
  const int top = std::clamp(pad + 34 + cascade_y, 0, std::max(0, viewport.height - height));
  return {left, top};
}

void ClampGraphWindow(GraphWindowState& state, const mjrRect& viewport, int width, int height) {
  state.left = std::clamp(state.left, 0, std::max(0, viewport.width - width));
  state.top = std::clamp(state.top, 0, std::max(0, viewport.height - height));
}

GraphWindowUi ComputeGraphWindowUi(ViewerState& viewer, int graph_index, const mjrRect& viewport, const DockUi& dock) {
  GraphWindowUi ui{};
  ui.graph_index = graph_index;
  if (graph_index < 0 || graph_index >= static_cast<int>(viewer.graph_windows.size())) {
    return ui;
  }

  GraphWindowState& state = viewer.graph_windows[static_cast<std::size_t>(graph_index)];
  ui.visible = state.visible;
  ui.minimized = state.minimized;
  ui.show_selector = state.show_selector;
  ui.y_auto = state.y_auto;
  ui.serial = state.serial;
  ui.x_seconds = state.x_seconds;
  ui.y_manual_limit = state.y_manual_limit;
  if (!ui.visible) {
    return ui;
  }

  const int width = GraphWindowDefaultWidth(viewer, viewport);
  const int header_height = WindowHeaderHeight(viewer.con);
  const int controls_height = GraphControlsHeight(viewer.con);
  const int height = ui.minimized ? header_height : GraphWindowDefaultHeight(viewer, viewport);
  if (!state.user_moved) {
    const auto [left, top] = DefaultGraphWindowTopLeft(viewer, graph_index, viewport, dock, width, height);
    state.left = left;
    state.top = top;
  }
  ClampGraphWindow(state, viewport, width, height);

  ui.rect = MakeTopLeftRect(viewport, state.left, state.top, width, height);
  ui.header_rect = mjrRect{ui.rect.left, ui.rect.bottom + ui.rect.height - header_height, ui.rect.width, header_height};

  const int button_size = std::max(14, header_height - 10);
  const int button_bottom = ui.header_rect.bottom + (ui.header_rect.height - button_size) / 2;
  const int close_left = ui.header_rect.left + ui.header_rect.width - button_size - 6;
  const int minimize_left = close_left - button_size - 4;
  ui.close_button = mjrRect{close_left, button_bottom, button_size, button_size};
  ui.minimize_button = mjrRect{minimize_left, button_bottom, button_size, button_size};

  const int icon_size = std::max(12, header_height - 12);
  ui.icon_rect = mjrRect{ui.header_rect.left + 8, ui.header_rect.bottom + (ui.header_rect.height - icon_size) / 2, icon_size,
                         icon_size};
  const int title_left = ui.icon_rect.left + ui.icon_rect.width + 8;
  const int title_width = std::max(0, minimize_left - title_left - 8);
  ui.title_rect = mjrRect{title_left, ui.header_rect.bottom + ui.header_rect.height / 2, title_width,
                          std::max(0, ui.header_rect.height / 2 - 2)};
  ui.subtitle_rect = mjrRect{title_left, ui.header_rect.bottom + 2, title_width, std::max(0, ui.header_rect.height / 2 - 4)};

  if (ui.minimized) {
    return ui;
  }

  ui.controls_rect = mjrRect{ui.rect.left, ui.rect.bottom + ui.rect.height - header_height - controls_height, ui.rect.width,
                             controls_height};
  const mjrRect body_rect = mjrRect{ui.rect.left, ui.rect.bottom, ui.rect.width,
                                    std::max(0, ui.rect.height - header_height - controls_height)};
  const int control_pad_x = 8;
  const int control_pad_y = 4;
  const int control_gap = 4;
  const int control_button_width = GraphControlButtonWidth(viewer.con);
  const int value_button_width = GraphValueButtonWidth(viewer.con);
  const int control_button_height =
      std::max(16, (ui.controls_rect.height - 2 * control_pad_y - control_gap) / 2);
  const int top_row_bottom =
      ui.controls_rect.bottom + ui.controls_rect.height - control_pad_y - control_button_height;
  const int bottom_row_bottom = ui.controls_rect.bottom + control_pad_y;

  int top_right = ui.controls_rect.left + ui.controls_rect.width - control_pad_x;
  ui.y_plus_button = mjrRect{top_right - control_button_width, top_row_bottom, control_button_width, control_button_height};
  top_right -= control_button_width + control_gap;
  ui.y_minus_button = mjrRect{top_right - control_button_width, top_row_bottom, control_button_width, control_button_height};
  top_right -= control_button_width + control_gap;
  ui.y_value_button = mjrRect{top_right - value_button_width, top_row_bottom, value_button_width, control_button_height};
  top_right -= value_button_width + control_gap;
  ui.y_mode_button = mjrRect{top_right - control_button_width, top_row_bottom, control_button_width, control_button_height};
  top_right -= control_button_width + control_gap;
  ui.selector_toggle_button = mjrRect{top_right - control_button_width, top_row_bottom, control_button_width,
                                      control_button_height};

  int bottom_right = ui.controls_rect.left + ui.controls_rect.width - control_pad_x;
  ui.x_plus_button = mjrRect{bottom_right - control_button_width, bottom_row_bottom, control_button_width,
                             control_button_height};
  bottom_right -= control_button_width + control_gap;
  ui.x_minus_button = mjrRect{bottom_right - control_button_width, bottom_row_bottom, control_button_width,
                              control_button_height};
  bottom_right -= control_button_width + control_gap;
  ui.x_value_button = mjrRect{bottom_right - value_button_width, bottom_row_bottom, value_button_width,
                              control_button_height};

  const int pane_gap = 8;
  const int selector_width = ui.show_selector ? std::clamp(body_rect.width * 30 / 100, 140, 210) : 0;
  const int selector_header_height = WindowColumnsHeight(viewer.con);
  const int selector_footer_height = WindowFooterHeight(viewer.con);
  ui.selector_rect = mjrRect{body_rect.left, body_rect.bottom, selector_width, body_rect.height};
  ui.selector_header_rect =
      mjrRect{ui.selector_rect.left, ui.selector_rect.bottom + ui.selector_rect.height - selector_header_height, ui.selector_rect.width,
              selector_header_height};
  ui.selector_footer_rect = mjrRect{ui.selector_rect.left, ui.selector_rect.bottom, ui.selector_rect.width, selector_footer_height};
  ui.selector_body_rect = mjrRect{ui.selector_rect.left, ui.selector_footer_rect.bottom + ui.selector_footer_rect.height,
                                  ui.selector_rect.width,
                                  std::max(0, ui.selector_header_rect.bottom - (ui.selector_footer_rect.bottom +
                                                                               ui.selector_footer_rect.height))};

  const int row_height = WindowRowHeight(viewer.con);
  const int row_gap = WindowRowGap(viewer.con);
  const mjrRect selector_rows = InsetRect(ui.selector_body_rect, 6, 6);
  const int rows_per_page = std::max(1, (selector_rows.height + row_gap) / std::max(1, row_height + row_gap));
  ui.total_pages = PageCount(viewer.model ? viewer.model->njnt : 0, rows_per_page);
  state.joint_page = std::clamp(state.joint_page, 0, ui.total_pages - 1);
  ui.page_index = state.joint_page;
  const int start = ui.page_index * rows_per_page;
  const int end = std::min(viewer.model ? viewer.model->njnt : 0, start + rows_per_page);
  ui.range_start = (end > start) ? start + 1 : 0;
  ui.range_end = end;
  state.selected_joint = std::clamp(state.selected_joint, 0, std::max(0, (viewer.model ? viewer.model->njnt : 1) - 1));
  ui.selected_joint = state.selected_joint;

  if (ui.show_selector) {
    int row_bottom = selector_rows.bottom + selector_rows.height - row_height;
    for (int jid = start; jid < end; ++jid) {
      ui.visible_joint_ids.push_back(jid);
      ui.joint_rows.push_back(mjrRect{selector_rows.left, row_bottom, selector_rows.width, row_height});
      row_bottom -= row_height + row_gap;
    }
  }

  const int footer_button_height = std::max(16, ui.selector_footer_rect.height - 8);
  const int footer_button_width = std::clamp(EstimateTextWidth(viewer.con, "NEXT") + viewer.con.charHeight / 3, 38, 62);
  const int footer_bottom = ui.selector_footer_rect.bottom + (ui.selector_footer_rect.height - footer_button_height) / 2;
  ui.prev_button = mjrRect{ui.selector_footer_rect.left + 6, footer_bottom, footer_button_width, footer_button_height};
  ui.next_button = mjrRect{ui.selector_footer_rect.left + ui.selector_footer_rect.width - footer_button_width - 6, footer_bottom,
                           footer_button_width, footer_button_height};

  const int charts_left = ui.show_selector ? (ui.selector_rect.left + ui.selector_rect.width + pane_gap) : body_rect.left;
  const int charts_width = ui.show_selector ? std::max(0, body_rect.width - ui.selector_rect.width - pane_gap) : body_rect.width;
  const mjrRect charts_rect = mjrRect{charts_left, body_rect.bottom, charts_width, body_rect.height};
  const mjrRect charts_inner = InsetRect(charts_rect, 8, 8);
  const int graph_gap = 8;
  const int graph_height = std::max(56, (charts_inner.height - 2 * graph_gap) / 3);
  int graph_bottom = charts_inner.bottom + charts_inner.height - graph_height;
  for (int metric = 0; metric < 3; ++metric) {
    ui.graph_rects[metric] = mjrRect{charts_inner.left, graph_bottom, charts_inner.width, graph_height};
    graph_bottom -= graph_height + graph_gap;
  }

  return ui;
}

DesktopUi ComputeDesktopUi(ViewerState& viewer, const mjrRect& viewport) {
  DesktopUi ui{};
  const bool viewport_changed =
      viewport.width != viewer.last_viewport_width || viewport.height != viewer.last_viewport_height;
  if (viewport_changed && viewer.last_viewport_width > 0 && viewer.last_viewport_height > 0) {
    for (FloatingWindowState& state : viewer.floating_windows) {
      if (!state.user_moved) {
        continue;
      }
      state.left = static_cast<int>(
          std::lround(static_cast<double>(state.left) * viewport.width / std::max(1, viewer.last_viewport_width)));
      state.top = static_cast<int>(
          std::lround(static_cast<double>(state.top) * viewport.height / std::max(1, viewer.last_viewport_height)));
    }
    for (GraphWindowState& state : viewer.graph_windows) {
      if (!state.user_moved) {
        continue;
      }
      state.left = static_cast<int>(
          std::lround(static_cast<double>(state.left) * viewport.width / std::max(1, viewer.last_viewport_width)));
      state.top = static_cast<int>(
          std::lround(static_cast<double>(state.top) * viewport.height / std::max(1, viewer.last_viewport_height)));
    }
  }

  const int min_dim = std::min(viewport.width, viewport.height);
  const int pad = std::clamp(min_dim / 140, 6, 12);
  const int dock_width = DockWidth(viewer.con);
  ui.dock.rect = mjrRect{viewport.left + pad, viewport.bottom + pad, dock_width, std::max(0, viewport.height - 2 * pad)};

  const int button_gap = std::clamp(viewer.con.charHeight / 4, 4, 8);
  const int button_size = std::min(ui.dock.rect.width - 12, std::clamp(viewer.con.charHeight * 2 + 6, 36, 52));
  int button_bottom = ui.dock.rect.bottom + ui.dock.rect.height - button_size - 8;
  for (int category = 0; category < kCategoryCount; ++category) {
    if (category == kCategoryStatus) {
      button_bottom -= button_gap;
    }
    ui.dock.buttons[category] = mjrRect{ui.dock.rect.left + (ui.dock.rect.width - button_size) / 2, button_bottom, button_size,
                                        button_size};
    button_bottom -= button_size + button_gap;
  }

  for (int category = 0; category < kCategoryCount; ++category) {
    ui.windows[category] = ComputeFloatingWindowUi(viewer, category, viewport, ui.dock);
  }
  ui.graph_windows.reserve(viewer.graph_windows.size());
  for (int graph_index = 0; graph_index < static_cast<int>(viewer.graph_windows.size()); ++graph_index) {
    ui.graph_windows.push_back(ComputeGraphWindowUi(viewer, graph_index, viewport, ui.dock));
  }

  viewer.last_viewport_width = viewport.width;
  viewer.last_viewport_height = viewport.height;
  return ui;
}

DesktopUi CurrentDesktopUi(GLFWwindow* window) {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return ComputeDesktopUi(*g_viewer, mjrRect{0, 0, width, height});
}

void DrawWindowButton(const ViewerState& viewer, const mjrRect& rect, const char* text, bool active, bool muted = false) {
  const float bg = active ? 0.20f : 0.13f;
  const float accent = active ? 0.68f : 0.32f;
  const float alpha = muted ? 0.46f : 0.90f;
  mjr_label(rect, mjFONT_NORMAL, text, bg, bg, bg, alpha, accent, accent, accent, &viewer.con);
}

std::string FloatingWindowSubtitle(const FloatingWindowUi& ui) {
  std::ostringstream oss;
  if (ui.total_items == 0) {
    oss << "0 items";
  } else {
    oss << ui.range_start << "-" << ui.range_end << " / " << ui.total_items;
  }
  return oss.str();
}

const char* PauseButtonText(const ViewerState& viewer) { return viewer.paused ? "RUN" : "PAUSE"; }

constexpr std::array<std::array<float, 3>, 6> kJointGraphColors = {{
    {0.31f, 0.70f, 0.95f},
    {0.98f, 0.61f, 0.22f},
    {0.51f, 0.82f, 0.42f},
    {0.92f, 0.34f, 0.34f},
    {0.76f, 0.52f, 0.95f},
    {0.95f, 0.80f, 0.28f},
}};

void DrawPlotDot(int x, int y, float r, float g, float b, float a) {
  mjr_rectangle(mjrRect{x, y, 2, 2}, r, g, b, a);
}

void DrawPlotSegment(int x0, int y0, int x1, int y1, float r, float g, float b, float a) {
  const int dx = x1 - x0;
  const int dy = y1 - y0;
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps <= 0) {
    DrawPlotDot(x0, y0, r, g, b, a);
    return;
  }

  for (int step = 0; step <= steps; ++step) {
    const float t = static_cast<float>(step) / static_cast<float>(steps);
    const int x = static_cast<int>(std::lround(x0 + dx * t));
    const int y = static_cast<int>(std::lround(y0 + dy * t));
    DrawPlotDot(x, y, r, g, b, a);
  }
}

void DrawJointMetricGraph(const ViewerState& viewer, const mjrRect& rect, const char* label, int metric,
                          const std::vector<int>& joint_ids, double x_seconds, bool y_auto, double manual_y_limit) {
  if (rect.width <= 0 || rect.height <= 0) {
    return;
  }

  mjr_rectangle(rect, 0.04f, 0.06f, 0.08f, 0.60f);
  const int title_height = std::clamp(viewer.con.charHeight + 4, 14, 22);
  const int xaxis_height = std::clamp(viewer.con.charHeight + 4, 12, 20);
  const int yaxis_width = std::clamp(EstimateTextWidth(viewer.con, "-100.0") + 8, 40, 62);
  const int content_gap = 4;
  const mjrRect title_rect = {rect.left + 6, rect.bottom + rect.height - title_height - 3, rect.width - 12, title_height};
  const int plot_bottom = rect.bottom + xaxis_height + 6;
  const int plot_top = title_rect.bottom - content_gap;
  const mjrRect plot_rect = {rect.left + yaxis_width, plot_bottom, std::max(0, rect.width - yaxis_width - 8),
                             std::max(0, plot_top - plot_bottom)};
  const mjrRect ymax_rect = {rect.left + 2, std::max(rect.bottom, plot_rect.bottom + plot_rect.height - viewer.con.charHeight),
                             std::max(0, yaxis_width - 4), viewer.con.charHeight + 2};
  const mjrRect ymin_rect = {rect.left + 2, plot_rect.bottom - viewer.con.charHeight / 3, std::max(0, yaxis_width - 4),
                             viewer.con.charHeight + 2};
  const mjrRect xleft_rect = {plot_rect.left, rect.bottom + 1, std::min(50, plot_rect.width), xaxis_height};
  const mjrRect xright_rect = {std::max(plot_rect.left, plot_rect.left + plot_rect.width - 40), rect.bottom + 1, 40,
                               xaxis_height};
  if (plot_rect.width <= 4 || plot_rect.height <= 4 || viewer.joint_history_count <= 0) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, title_rect, label, "", &viewer.con);
    return;
  }

  const float latest_time = JointHistoryTimeValue(viewer, viewer.joint_history_count - 1);
  const float earliest_time = std::max(0.0f, latest_time - static_cast<float>(std::max(0.1, x_seconds)));
  int start_sample = 0;
  while (start_sample + 1 < viewer.joint_history_count && JointHistoryTimeValue(viewer, start_sample) < earliest_time) {
    ++start_sample;
  }

  float min_value = 0.0f;
  float max_value = 0.0f;
  bool first = true;
  for (int jid : joint_ids) {
    for (int sample = start_sample; sample < viewer.joint_history_count; ++sample) {
      const float value = JointHistoryValue(viewer, jid, metric, sample);
      if (first) {
        min_value = max_value = value;
        first = false;
      } else {
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
      }
    }
  }

  if (first) {
    min_value = -1.0f;
    max_value = 1.0f;
  } else if (!y_auto) {
    const float limit = std::max(0.05f, static_cast<float>(manual_y_limit));
    min_value = -limit;
    max_value = limit;
  } else if (std::abs(max_value - min_value) < 1.0e-4f) {
    const float pad = std::max(0.05f, std::abs(max_value) * 0.15f);
    min_value -= pad;
    max_value += pad;
  } else {
    const float pad = (max_value - min_value) * 0.08f;
    min_value -= pad;
    max_value += pad;
  }

  const std::string range_text = FormatSigned(min_value, 1) + " .. " + FormatSigned(max_value, 1);

  for (int gx = 0; gx <= 4; ++gx) {
    const int x = plot_rect.left + gx * (plot_rect.width - 1) / 4;
    mjr_rectangle(mjrRect{x, plot_rect.bottom, 1, plot_rect.height}, 0.18f, 0.22f, 0.26f, gx == 0 ? 0.65f : 0.34f);
  }
  for (int gy = 0; gy <= 4; ++gy) {
    const int y = plot_rect.bottom + gy * (plot_rect.height - 1) / 4;
    mjr_rectangle(mjrRect{plot_rect.left, y, plot_rect.width, 1}, 0.18f, 0.22f, 0.26f, gy == 0 ? 0.65f : 0.34f);
  }

  const float zero_y = std::clamp((0.0f - min_value) / std::max(1.0e-6f, max_value - min_value), 0.0f, 1.0f);
  const int baseline = plot_rect.bottom + static_cast<int>(std::lround((1.0f - zero_y) * (plot_rect.height - 1)));
  mjr_rectangle(mjrRect{plot_rect.left, baseline, plot_rect.width, 1}, 0.20f, 0.26f, 0.32f, 0.55f);

  for (std::size_t series = 0; series < joint_ids.size(); ++series) {
    const std::array<float, 3>& color = kJointGraphColors[series % kJointGraphColors.size()];
    int prev_x = 0;
    int prev_y = 0;
    bool has_prev = false;
    for (int sample = start_sample; sample < viewer.joint_history_count; ++sample) {
      const float value = JointHistoryValue(viewer, joint_ids[series], metric, sample);
      const float t0 = JointHistoryTimeValue(viewer, start_sample);
      const float t1 = JointHistoryTimeValue(viewer, viewer.joint_history_count - 1);
      const float x_norm = (std::abs(t1 - t0) < 1.0e-6f) ? 1.0f : (JointHistoryTimeValue(viewer, sample) - t0) / (t1 - t0);
      const float y_norm = (value - min_value) / std::max(1.0e-6f, max_value - min_value);
      const int x = plot_rect.left + static_cast<int>(std::lround(x_norm * (plot_rect.width - 1)));
      const int y = plot_rect.bottom + static_cast<int>(std::lround(y_norm * (plot_rect.height - 1)));
      if (has_prev) {
        DrawPlotSegment(prev_x, prev_y, x, y, color[0], color[1], color[2], 0.92f);
      } else {
        DrawPlotDot(x, y, color[0], color[1], color[2], 0.92f);
      }
      prev_x = x;
      prev_y = y;
      has_prev = true;
    }
  }

  const int range_width = std::min(title_rect.width / 2, EstimateTextWidth(viewer.con, range_text) + 6);
  const mjrRect range_rect = {title_rect.left + title_rect.width - range_width, title_rect.bottom, range_width, title_rect.height};
  const mjrRect label_rect = {title_rect.left, title_rect.bottom, std::max(0, title_rect.width - range_width - 4), title_rect.height};
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, label_rect, label, "", &viewer.con);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, range_rect, range_text.c_str(), "", &viewer.con);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ymax_rect, FormatSigned(max_value, 1).c_str(), "", &viewer.con);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ymin_rect, FormatSigned(min_value, 1).c_str(), "", &viewer.con);
  std::ostringstream xspan;
  xspan << "-" << FormatFixed(x_seconds, 1) << "s";
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, xleft_rect, xspan.str().c_str(), "", &viewer.con);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, xright_rect, "now", "", &viewer.con);
}

std::string GraphWindowSubtitle(const ViewerState& viewer, const GraphWindowUi& ui) {
  if (!viewer.model || ui.selected_joint < 0 || ui.selected_joint >= viewer.model->njnt) {
    return "joint ?";
  }
  std::ostringstream oss;
  oss << ui.selected_joint << "  " << TruncateText(SafeName(viewer.model, mjOBJ_JOINT, ui.selected_joint, "joint"), 28);
  return oss.str();
}

void DrawGraphWindow(const ViewerState& viewer, const GraphWindowUi& ui) {
  if (!ui.visible || ui.rect.width <= 0 || ui.rect.height <= 0) {
    return;
  }

  const bool focused = viewer.focused_graph_window == ui.graph_index;
  const mjrRect shadow = {ui.rect.left + 4, ui.rect.bottom - 4, ui.rect.width, ui.rect.height};
  mjr_rectangle(shadow, 0.0f, 0.0f, 0.0f, 0.16f);
  mjr_rectangle(ui.rect, 0.02f, 0.04f, 0.06f, 0.40f);
  mjr_rectangle(ui.header_rect, 0.08f, 0.12f, 0.16f, focused ? 0.94f : 0.88f);

  const UiIconTexture& icon = viewer.icons[kIconJoints];
  if (!icon.bgra.empty()) {
    DrawIconTexture(icon, ui.icon_rect, viewer.con, 0.08f, 0.12f, 0.16f, focused ? 1.0f : 0.86f);
  }

  std::string title = "Graph " + std::to_string(ui.serial);
  const std::string subtitle = GraphWindowSubtitle(viewer, ui);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.title_rect, title.c_str(), "", &viewer.con);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.subtitle_rect, subtitle.c_str(), "", &viewer.con);
  DrawWindowButton(viewer, ui.minimize_button, "_", ui.minimized);
  DrawWindowButton(viewer, ui.close_button, "x", false);

  if (ui.minimized) {
    return;
  }

  mjr_rectangle(ui.controls_rect, 0.07f, 0.09f, 0.11f, 0.70f);
  DrawWindowButton(viewer, ui.selector_toggle_button, ui.show_selector ? "LIST" : "SHOW", ui.show_selector);
  DrawWindowButton(viewer, ui.y_mode_button, ui.y_auto ? "AUTO" : "MAN", ui.y_auto);
  const bool editing_x = viewer.editing_graph_window == ui.graph_index && viewer.editing_graph_field == kGraphEditXSeconds;
  const bool editing_y = viewer.editing_graph_window == ui.graph_index && viewer.editing_graph_field == kGraphEditYLimit;
  const std::string x_value_text = editing_x ? viewer.graph_edit_buffer : ("X " + FormatFixed(ui.x_seconds, 1) + "s");
  const std::string y_value_text = editing_y ? viewer.graph_edit_buffer : ("Y " + FormatFixed(ui.y_manual_limit, 1));
  DrawWindowButton(viewer, ui.y_value_button, y_value_text.c_str(), editing_y || !ui.y_auto, false);
  DrawWindowButton(viewer, ui.y_minus_button, "Y-", false, ui.y_auto);
  DrawWindowButton(viewer, ui.y_plus_button, "Y+", false, ui.y_auto);
  DrawWindowButton(viewer, ui.x_value_button, x_value_text.c_str(), editing_x, false);
  DrawWindowButton(viewer, ui.x_minus_button, "X-", false);
  DrawWindowButton(viewer, ui.x_plus_button, "X+", false);

  if (editing_x || editing_y) {
    const mjrRect controls_info = {ui.controls_rect.left + 8, ui.controls_rect.bottom + 4,
                                   std::max(0, ui.x_value_button.left - ui.controls_rect.left - 14),
                                   std::max(0, ui.controls_rect.height / 2 - 4)};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, controls_info, "ENTER apply  ESC cancel", "", &viewer.con);
  }

  if (ui.show_selector) {
    mjr_rectangle(ui.selector_rect, 0.05f, 0.07f, 0.09f, 0.64f);
    mjr_rectangle(ui.selector_header_rect, 0.09f, 0.11f, 0.13f, 0.82f);
    mjr_rectangle(ui.selector_footer_rect, 0.07f, 0.10f, 0.12f, 0.72f);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.selector_header_rect, "JOINTS", "", &viewer.con);

    for (std::size_t i = 0; i < ui.joint_rows.size() && i < ui.visible_joint_ids.size(); ++i) {
      const int jid = ui.visible_joint_ids[i];
      const bool selected = jid == ui.selected_joint;
      const float shade = selected ? 0.16f : ((i % 2 == 0) ? 0.10f : 0.08f);
      mjr_rectangle(ui.joint_rows[i], shade, shade, shade, selected ? 0.84f : 0.66f);
      std::ostringstream label;
      label << jid << "  " << TruncateText(SafeName(viewer.model, mjOBJ_JOINT, jid, "joint"), 18);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.joint_rows[i], label.str().c_str(), "", &viewer.con);
    }

    DrawWindowButton(viewer, ui.prev_button, "<", false, ui.page_index <= 0);
    DrawWindowButton(viewer, ui.next_button, ">", false, ui.page_index + 1 >= ui.total_pages);
    std::ostringstream page_label;
    page_label << ui.range_start << "-" << ui.range_end << " / " << viewer.model->njnt;
    const mjrRect page_rect = {ui.prev_button.left + ui.prev_button.width + 6, ui.selector_footer_rect.bottom,
                               std::max(0, ui.next_button.left - (ui.prev_button.left + ui.prev_button.width + 12)),
                               ui.selector_footer_rect.height};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, page_rect, page_label.str().c_str(), "", &viewer.con);
  }

  const std::vector<int> selected_joint_ids = {ui.selected_joint};
  DrawJointMetricGraph(viewer, ui.graph_rects[0], "Q", 0, selected_joint_ids, ui.x_seconds, ui.y_auto, ui.y_manual_limit);
  DrawJointMetricGraph(viewer, ui.graph_rects[1], "V", 1, selected_joint_ids, ui.x_seconds, ui.y_auto, ui.y_manual_limit);
  DrawJointMetricGraph(viewer, ui.graph_rects[2], "TAU", 2, selected_joint_ids, ui.x_seconds, ui.y_auto, ui.y_manual_limit);
}

void DrawFloatingWindow(const ViewerState& viewer, const FloatingWindowUi& ui) {
  if (!ui.visible || ui.rect.width <= 0 || ui.rect.height <= 0) {
    return;
  }

  const std::array<UiIconId, kCategoryCount> dock_icons = {
      kIconFavorites, kIconStatus, kIconScene, kIconKeys, kIconJoints, kIconActuators};
  const bool focused = viewer.active_category == ui.category;

  const mjrRect shadow = {ui.rect.left + 4, ui.rect.bottom - 4, ui.rect.width, ui.rect.height};
  mjr_rectangle(shadow, 0.0f, 0.0f, 0.0f, 0.16f);
  mjr_rectangle(ui.rect, 0.02f, 0.04f, 0.06f, 0.40f);
  mjr_rectangle(ui.header_rect, 0.08f, 0.12f, 0.16f, focused ? 0.94f : 0.88f);

  if (!ui.minimized) {
    mjr_rectangle(ui.columns_rect, 0.10f, 0.10f, 0.10f, 0.78f);
    mjr_rectangle(ui.footer_rect, 0.07f, 0.10f, 0.12f, 0.72f);
  }

  const UiIconTexture& icon = viewer.icons[dock_icons[ui.category]];
  if (!icon.bgra.empty()) {
    DrawIconTexture(icon, ui.icon_rect, viewer.con, 0.08f, 0.12f, 0.16f, focused ? 1.0f : 0.86f);
  }

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.title_rect, kCategoryTitles[ui.category], "", &viewer.con);
  const std::string subtitle = FloatingWindowSubtitle(ui);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.subtitle_rect, subtitle.c_str(), "", &viewer.con);

  DrawWindowButton(viewer, ui.minimize_button, "_", ui.minimized);
  DrawWindowButton(viewer, ui.close_button, "x", false);

  if (ui.minimized) {
    return;
  }

  const TableSpec spec = GetTableSpec(ui.category);
  const int favorite_width = std::clamp(EstimateTextWidth(viewer.con, "*") + viewer.con.charHeight / 2, 18, 30);
  const std::array<mjrRect, 6> header_cells = ComputeColumnRects(ui.columns_rect, favorite_width, spec);
  const mjrRect star_header = {ui.columns_rect.left + 6, ui.columns_rect.bottom + 4, favorite_width,
                               std::max(0, ui.columns_rect.height - 8)};
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, star_header, "*", "", &viewer.con);
  for (int i = 0; i < spec.column_count; ++i) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, header_cells[i], spec.headers[i], "", &viewer.con);
  }

  int visible_index = 0;
  for (const DrawerEntryUi& entry : ui.entries) {
    const float shade = (visible_index % 2 == 0) ? 0.11f : 0.08f;
    mjr_rectangle(entry.rect, shade, shade, shade, 0.66f);
    if (entry.item.can_favorite) {
      const UiIconTexture& favorite_icon = viewer.icons[entry.item.favorite ? kIconFavoriteOn : kIconFavoriteOff];
      if (!favorite_icon.bgra.empty()) {
        DrawIconTexture(favorite_icon, entry.favorite_button, viewer.con, shade, shade, shade, entry.item.favorite ? 1.0f : 0.70f);
      } else {
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, entry.favorite_button, entry.item.favorite ? "*" : ".", "", &viewer.con);
      }
    }

    const std::array<mjrRect, 6> cells = ComputeColumnRects(entry.rect, entry.favorite_button.width, spec);
    for (int col = 0; col < spec.column_count && col < entry.item.column_count; ++col) {
      const std::string cell_text = FitTextToWidth(viewer.con, entry.item.columns[col], cells[col].width - 8);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, cells[col], cell_text.c_str(), "", &viewer.con);
    }
    ++visible_index;
  }

  const mjrRect footer_center = {ui.footer_rect.left + 110, ui.footer_rect.bottom, std::max(0, ui.footer_rect.width - 220),
                                 ui.footer_rect.height};
  const std::string footer_text =
      ui.total_pages > 1 ? ("PAGE " + std::to_string(ui.page_index + 1) + " / " + std::to_string(ui.total_pages))
                         : "READY";
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, footer_center, footer_text.c_str(), "", &viewer.con);

  if (ui.show_page_controls) {
    DrawWindowButton(viewer, ui.prev_button, "<", false);
    DrawWindowButton(viewer, ui.next_button, ">", false);
  }
  if (ui.show_favorites_button) {
    DrawWindowButton(viewer, ui.favorite_filter_button, viewer.favorites_only[ui.category] ? "FAV*" : "FAV",
                     viewer.favorites_only[ui.category]);
  }
  if (ui.show_graph_button) {
    DrawWindowButton(viewer, ui.graph_button, "GRAPH+", false);
  }
  if (ui.show_status_actions) {
    DrawWindowButton(viewer, ui.pause_button, PauseButtonText(viewer), viewer.paused);
    DrawWindowButton(viewer, ui.step_button, "STEP", false);
    DrawWindowButton(viewer, ui.task_reset_button, "TASK", false);
    DrawWindowButton(viewer, ui.reset_button, "RESET", false);
  }
}

void DrawDock(const ViewerState& viewer, const DockUi& dock) {
  const std::array<UiIconId, kCategoryCount> dock_icons = {
      kIconFavorites, kIconStatus, kIconScene, kIconKeys, kIconJoints, kIconActuators};
  mjr_rectangle(dock.rect, 0.06f, 0.08f, 0.10f, 0.76f);

  for (int category = 0; category < kCategoryCount; ++category) {
    const bool visible = viewer.floating_windows[category].visible;
    const bool focused = viewer.active_category == category && visible;
    const float bg = focused ? 0.14f : (visible ? 0.12f : 0.08f);
    const float alpha = focused ? 0.94f : (visible ? 0.80f : 0.62f);
    mjr_label(dock.buttons[category], mjFONT_NORMAL, "", bg, bg, bg, alpha, 0.94f, 0.96f, 0.99f, &viewer.con);

    if (visible) {
      const mjrRect accent = {dock.buttons[category].left - 3, dock.buttons[category].bottom + 4, 2,
                              std::max(0, dock.buttons[category].height - 8)};
      mjr_rectangle(accent, 0.27f, 0.61f, 0.85f, 0.94f);
    }

    const int icon_size = std::max(16, std::min(dock.buttons[category].width, dock.buttons[category].height) - 12);
    const mjrRect icon_rect = {dock.buttons[category].left + (dock.buttons[category].width - icon_size) / 2,
                               dock.buttons[category].bottom + (dock.buttons[category].height - icon_size) / 2, icon_size,
                               icon_size};
    if (!viewer.icons[dock_icons[category]].bgra.empty()) {
      DrawIconTexture(viewer.icons[dock_icons[category]], icon_rect, viewer.con, bg, bg, bg, visible ? 1.0f : 0.72f);
    } else {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, dock.buttons[category], kCategoryIcons[category], "", &viewer.con);
    }
  }
}

void DrawDesktopUi(const ViewerState& viewer, const DesktopUi& ui) {
  for (int category : viewer.window_order) {
    DrawFloatingWindow(viewer, ui.windows[category]);
  }
  for (int graph_index : viewer.graph_window_order) {
    if (graph_index < 0 || graph_index >= static_cast<int>(ui.graph_windows.size())) {
      continue;
    }
    DrawGraphWindow(viewer, ui.graph_windows[static_cast<std::size_t>(graph_index)]);
  }
  DrawDock(viewer, ui.dock);
}

NavigationUi ComputeNavigationUi(ViewerState& viewer, const mjrRect& viewport) {
  NavigationUi ui{};
  const int min_dim = std::min(viewport.width, viewport.height);
  const int pad = std::clamp(min_dim / 125, 6, 14);
  const int gap = std::clamp(pad - 1, 4, 10);
  const int dock_width = std::clamp(viewer.con.charHeight * 2 + 24, 54, 74);
  const bool wide_table = viewer.active_category == kCategoryJoints || viewer.active_category == kCategoryActuators;
  const int drawer_width =
      wide_table ? std::clamp(viewport.width * 42 / 100, 460, 760) : std::clamp(viewport.width * 30 / 100, 340, 540);
  const int nav_height = std::max(0, viewport.height - 2 * pad);

  ui.dock_rect = mjrRect{viewport.left + pad, viewport.bottom + pad, dock_width, nav_height};
  ui.drawer_rect = mjrRect{ui.dock_rect.left + ui.dock_rect.width + gap, viewport.bottom + pad, drawer_width, nav_height};

  const int button_gap = gap;
  const int button_height = std::clamp(viewer.con.charHeight + 14, 28, 40);
  int button_bottom = ui.dock_rect.bottom + ui.dock_rect.height - button_height - gap;
  for (int category = 0; category < kCategoryCount; ++category) {
    if (category == kCategoryStatus) {
      button_bottom -= gap;
    }
    ui.dock_buttons[category] = mjrRect{ui.dock_rect.left + 6, button_bottom, ui.dock_rect.width - 12, button_height};
    button_bottom -= button_height + button_gap;
  }

  const int header_height = std::clamp(viewer.con.charHeight * 3 + 20, 56, 86);
  const int columns_height = std::clamp(viewer.con.charHeight + 12, 24, 34);
  const int footer_height = std::clamp(viewer.con.charHeight + 16, 26, 36);
  ui.header_rect = mjrRect{ui.drawer_rect.left, ui.drawer_rect.bottom + ui.drawer_rect.height - header_height,
                           ui.drawer_rect.width, header_height};
  ui.columns_rect = mjrRect{ui.drawer_rect.left, ui.header_rect.bottom - columns_height, ui.drawer_rect.width, columns_height};
  ui.footer_rect = mjrRect{ui.drawer_rect.left, ui.drawer_rect.bottom, ui.drawer_rect.width, footer_height};
  ui.body_rect = mjrRect{ui.drawer_rect.left, ui.footer_rect.bottom + ui.footer_rect.height, ui.drawer_rect.width,
                         std::max(0, ui.columns_rect.bottom - (ui.footer_rect.bottom + ui.footer_rect.height))};

  const int action_width = std::clamp(EstimateTextWidth(viewer.con, "PAUSE") + viewer.con.charHeight, 54, 86);
  const int action_height = std::clamp(viewer.con.charHeight + 8, 22, 30);
  int action_left = ui.header_rect.left + ui.header_rect.width - gap - action_width;
  for (int action = 0; action < kActionCount; ++action) {
    ui.action_buttons[action] = mjrRect{action_left, ui.header_rect.bottom + ui.header_rect.height - action_height - 8,
                                        action_width, action_height};
    action_left -= action_width + 6;
  }

  ui.title_rect = mjrRect{ui.header_rect.left + 12, ui.header_rect.bottom + ui.header_rect.height - action_height - 10,
                          std::max(0, ui.header_rect.width - 4 * action_width - 48), action_height};
  ui.category_rect = mjrRect{ui.header_rect.left + 12, ui.header_rect.bottom + 8,
                             std::max(0, ui.header_rect.width - 24), std::max(0, ui.header_rect.height - action_height - 18)};

  const std::vector<DrawerItem> items = BuildCategoryItems(viewer, viewer.active_category);
  ui.total_items = static_cast<int>(items.size());

  const int row_gap = std::clamp(viewer.con.charHeight / 6, 2, 6);
  const int row_height = std::clamp(viewer.con.charHeight + 14, 24, 38);
  const mjrRect rows_rect = InsetRect(ui.body_rect, 6, 6);
  const int rows_per_page = std::max(1, (rows_rect.height + row_gap) / std::max(1, row_height + row_gap));
  ui.total_pages = PageCount(ui.total_items, rows_per_page);

  int& page_index = viewer.category_pages[viewer.active_category];
  page_index = std::clamp(page_index, 0, ui.total_pages - 1);
  ui.page_index = page_index;

  const int start = page_index * rows_per_page;
  const int end = std::min(ui.total_items, start + rows_per_page);
  ui.range_start = ui.total_items == 0 ? 0 : start + 1;
  ui.range_end = end;

  int row_bottom = rows_rect.bottom + rows_rect.height - row_height;
  const int favorite_width = std::clamp(EstimateTextWidth(viewer.con, "*") + viewer.con.charHeight / 2, 18, 30);

  for (int item_index = start; item_index < end; ++item_index) {
    DrawerEntryUi entry;
    entry.item = items[static_cast<std::size_t>(item_index)];
    entry.rect = mjrRect{rows_rect.left, row_bottom, rows_rect.width, row_height};
    entry.favorite_button =
        mjrRect{entry.rect.left + 6, entry.rect.bottom + 4, favorite_width, std::max(0, row_height - 8)};
    ui.entries.push_back(std::move(entry));
    row_bottom -= row_height + row_gap;
  }

  const int page_button_width = std::clamp(EstimateTextWidth(viewer.con, "NEXT") + viewer.con.charHeight / 2, 46, 70);
  ui.prev_button = mjrRect{ui.footer_rect.left + 8, ui.footer_rect.bottom + 4, page_button_width, ui.footer_rect.height - 8};
  ui.next_button = mjrRect{ui.footer_rect.left + ui.footer_rect.width - page_button_width - 8, ui.footer_rect.bottom + 4,
                           page_button_width, ui.footer_rect.height - 8};
  return ui;
}

[[maybe_unused]] NavigationUi CurrentNavigationUi(GLFWwindow* window) {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return ComputeNavigationUi(*g_viewer, mjrRect{0, 0, width, height});
}

[[maybe_unused]] void DrawNavigationUi(const ViewerState& viewer, const NavigationUi& ui) {
  const TableSpec spec = GetTableSpec(viewer.active_category);
  const std::array<UiIconId, kCategoryCount> dock_icons = {
      kIconFavorites, kIconStatus, kIconScene, kIconKeys, kIconJoints, kIconActuators};

  mjr_rectangle(ui.dock_rect, 0.10f, 0.10f, 0.10f, 0.96f);
  mjr_rectangle(ui.drawer_rect, 0.15f, 0.15f, 0.15f, 0.94f);
  mjr_rectangle(ui.header_rect, 0.17f, 0.17f, 0.17f, 0.98f);
  mjr_rectangle(ui.columns_rect, 0.19f, 0.19f, 0.19f, 0.98f);
  mjr_rectangle(ui.footer_rect, 0.17f, 0.17f, 0.17f, 0.98f);

  const std::string explorer_title = "EXPLORER";
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.title_rect, explorer_title.c_str(), "", &viewer.con);

  std::ostringstream header_value;
  header_value << kCategoryTitles[viewer.active_category];
  if (ui.total_items > 0) {
    header_value << "   " << ui.range_start << "-" << ui.range_end << " / " << ui.total_items;
  }
  const std::string category_line = header_value.str();
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, ui.category_rect, category_line.c_str(), "", &viewer.con);

  for (int action = 0; action < kActionCount; ++action) {
    const bool highlighted =
        (action == kActionPause && viewer.paused) ||
        (action == kActionFavoritesOnly && viewer.favorites_only[viewer.active_category]);
    const float r = highlighted ? 0.00f : 0.15f;
    const float g = highlighted ? 0.48f : 0.20f;
    const float b = highlighted ? 0.80f : 0.24f;
    const float a = CategoryCanFilterFavorites(viewer.active_category) || action != kActionFavoritesOnly ? 0.96f : 0.30f;
    mjr_label(ui.action_buttons[action], mjFONT_NORMAL, ActionButtonText(viewer, action), r, g, b, a, 0.94f, 0.96f, 0.99f,
              &viewer.con);
  }

  for (int category = 0; category < kCategoryCount; ++category) {
    const bool active = viewer.active_category == category;
    const mjrRect accent = {ui.dock_buttons[category].left - 2, ui.dock_buttons[category].bottom, 3, ui.dock_buttons[category].height};
    if (active) {
      mjr_rectangle(accent, 0.00f, 0.48f, 0.80f, 0.98f);
    }
    const float r = active ? 0.20f : 0.14f;
    const float g = active ? 0.20f : 0.14f;
    const float b = active ? 0.20f : 0.14f;
    const float a = active ? 0.98f : 0.80f;
    mjr_label(ui.dock_buttons[category], mjFONT_NORMAL, "", r, g, b, a, 0.94f, 0.96f, 0.99f, &viewer.con);
    const int icon_size = std::max(16, std::min(ui.dock_buttons[category].width, ui.dock_buttons[category].height) - 12);
    const mjrRect icon_rect = {ui.dock_buttons[category].left + (ui.dock_buttons[category].width - icon_size) / 2,
                               ui.dock_buttons[category].bottom + (ui.dock_buttons[category].height - icon_size) / 2,
                               icon_size, icon_size};
    if (!viewer.icons[dock_icons[category]].bgra.empty()) {
      DrawIconTexture(viewer.icons[dock_icons[category]], icon_rect, viewer.con, r, g, b, active ? 1.0f : 0.78f);
    } else {
      mjr_label(ui.dock_buttons[category], mjFONT_NORMAL, kCategoryIcons[category], r, g, b, a, 0.94f, 0.96f, 0.99f,
                &viewer.con);
    }
  }

  const int favorite_width = std::clamp(EstimateTextWidth(viewer.con, "*") + viewer.con.charHeight / 2, 18, 30);
  const std::array<mjrRect, 6> header_cells = ComputeColumnRects(ui.columns_rect, favorite_width, spec);
  if (spec.column_count > 0) {
    const mjrRect star_header = {ui.columns_rect.left + 6, ui.columns_rect.bottom + 4, favorite_width,
                                 std::max(0, ui.columns_rect.height - 8)};
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, star_header, "*", "", &viewer.con);
    for (int i = 0; i < spec.column_count; ++i) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, header_cells[i], spec.headers[i], "", &viewer.con);
    }
  }

  int visible_index = 0;
  for (const DrawerEntryUi& entry : ui.entries) {
    const float shade = (visible_index % 2 == 0) ? 0.14f : 0.12f;
    mjr_rectangle(entry.rect, shade, shade, shade, 0.95f);
    if (entry.item.can_favorite) {
      const UiIconTexture& favorite_icon = viewer.icons[entry.item.favorite ? kIconFavoriteOn : kIconFavoriteOff];
      if (!favorite_icon.bgra.empty()) {
        DrawIconTexture(favorite_icon, entry.favorite_button, viewer.con, shade, shade, shade,
                        entry.item.favorite ? 1.0f : 0.72f);
      } else {
        mjr_label(entry.favorite_button, mjFONT_NORMAL, entry.item.favorite ? "*" : ".", 0.12f, 0.18f, 0.24f, 0.95f,
                  0.92f, 0.95f, 0.99f, &viewer.con);
      }
    }

    const std::array<mjrRect, 6> cells = ComputeColumnRects(entry.rect, entry.favorite_button.width, spec);
    for (int col = 0; col < spec.column_count && col < entry.item.column_count; ++col) {
      const std::string cell_text = FitTextToWidth(viewer.con, entry.item.columns[col], cells[col].width - 8);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, cells[col], cell_text.c_str(), "", &viewer.con);
    }
    ++visible_index;
  }

  const std::string footer_text = (ui.total_pages > 1)
                                      ? ("PAGE " + std::to_string(ui.page_index + 1) + " / " + std::to_string(ui.total_pages))
                                      : (viewer.favorites_only[viewer.active_category] ? "favorites only" : "all items");
  const mjrRect footer_center = mjrRect{ui.footer_rect.left + ui.prev_button.width + 14, ui.footer_rect.bottom,
                                        std::max(0, ui.footer_rect.width - 2 * ui.prev_button.width - 28),
                                        ui.footer_rect.height};
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, footer_center, footer_text.c_str(), "", &viewer.con);
  if (ui.total_pages > 1) {
    mjr_label(ui.prev_button, mjFONT_NORMAL, "<", 0.00f, 0.48f, 0.80f, 0.92f, 0.95f, 0.97f, 0.99f, &viewer.con);
    mjr_label(ui.next_button, mjFONT_NORMAL, ">", 0.00f, 0.48f, 0.80f, 0.92f, 0.95f, 0.97f, 0.99f, &viewer.con);
  }
}

bool IsCtrlPressed(GLFWwindow* window) {
  return glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
         glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

void ClearPerturb() {
  if (!g_viewer) {
    return;
  }
  g_viewer->pert.active = 0;
  g_viewer->pert.active2 = 0;
}

void CacheClothRootPose(ViewerState& viewer) {
  viewer.cloth_body_id = mj_name2id(viewer.model, mjOBJ_BODY, "cloth");
  viewer.cloth_flex_id = mj_name2id(viewer.model, mjOBJ_FLEX, "cloth");
  if (viewer.cloth_body_id < 0) {
    if (viewer.cloth_flex_id < 0) {
      return;
    }
  }

  if (viewer.cloth_flex_id >= 0) {
    const int rgba_adr = 4 * viewer.cloth_flex_id;
    for (int i = 0; i < 4; ++i) {
      viewer.cloth_default_rgba[static_cast<std::size_t>(i)] = viewer.model->flex_rgba[rgba_adr + i];
    }
  }

  if (viewer.cloth_body_id < 0) {
    return;
  }

  const int pos_adr = 3 * viewer.cloth_body_id;
  const int quat_adr = 4 * viewer.cloth_body_id;
  for (int i = 0; i < 3; ++i) {
    viewer.cloth_default_pos[static_cast<std::size_t>(i)] = viewer.model->body_pos[pos_adr + i];
  }
  for (int i = 0; i < 4; ++i) {
    viewer.cloth_default_quat[static_cast<std::size_t>(i)] = viewer.model->body_quat[quat_adr + i];
  }
}

void SetClothRootPose(ViewerState& viewer, mjtNum x, mjtNum y, mjtNum roll, mjtNum pitch, mjtNum yaw) {
  if (viewer.cloth_body_id < 0) {
    return;
  }

  const int pos_adr = 3 * viewer.cloth_body_id;
  const int quat_adr = 4 * viewer.cloth_body_id;
  viewer.model->body_pos[pos_adr + 0] = x;
  viewer.model->body_pos[pos_adr + 1] = y;
  viewer.model->body_pos[pos_adr + 2] = viewer.cloth_default_pos[2];

  const mjtNum half_roll = 0.5 * roll;
  const mjtNum half_pitch = 0.5 * pitch;
  const mjtNum half_yaw = 0.5 * yaw;
  const mjtNum cr = std::cos(half_roll);
  const mjtNum sr = std::sin(half_roll);
  const mjtNum cp = std::cos(half_pitch);
  const mjtNum sp = std::sin(half_pitch);
  const mjtNum cy = std::cos(half_yaw);
  const mjtNum sy = std::sin(half_yaw);

  viewer.model->body_quat[quat_adr + 0] = cr * cp * cy + sr * sp * sy;
  viewer.model->body_quat[quat_adr + 1] = sr * cp * cy - cr * sp * sy;
  viewer.model->body_quat[quat_adr + 2] = cr * sp * cy + sr * cp * sy;
  viewer.model->body_quat[quat_adr + 3] = cr * cp * sy - sr * sp * cy;
}

void SetClothColor(ViewerState& viewer, float r, float g, float b, float a) {
  if (viewer.cloth_flex_id < 0) {
    return;
  }

  const int rgba_adr = 4 * viewer.cloth_flex_id;
  viewer.model->flex_rgba[rgba_adr + 0] = r;
  viewer.model->flex_rgba[rgba_adr + 1] = g;
  viewer.model->flex_rgba[rgba_adr + 2] = b;
  viewer.model->flex_rgba[rgba_adr + 3] = a;
}

void RestoreDefaultClothRootPose(ViewerState& viewer) {
  if (viewer.cloth_body_id < 0) {
    return;
  }

  const int pos_adr = 3 * viewer.cloth_body_id;
  const int quat_adr = 4 * viewer.cloth_body_id;
  for (int i = 0; i < 3; ++i) {
    viewer.model->body_pos[pos_adr + i] = viewer.cloth_default_pos[static_cast<std::size_t>(i)];
  }
  for (int i = 0; i < 4; ++i) {
    viewer.model->body_quat[quat_adr + i] = viewer.cloth_default_quat[static_cast<std::size_t>(i)];
  }
}

void RestoreDefaultClothColor(ViewerState& viewer) {
  SetClothColor(viewer,
                viewer.cloth_default_rgba[0],
                viewer.cloth_default_rgba[1],
                viewer.cloth_default_rgba[2],
                viewer.cloth_default_rgba[3]);
}

void CacheTaskBoxes(ViewerState& viewer) {
  constexpr std::array<const char*, 2> kTaskBoxNames = {"box_1", "box_2"};
  constexpr std::array<const char*, 2> kTaskBoxGeomNames = {"box_1_geom", "box_2_geom"};
  for (int i = 0; i < 2; ++i) {
    const int body_id = mj_name2id(viewer.model, mjOBJ_BODY, kTaskBoxNames[static_cast<std::size_t>(i)]);
    const int geom_id = mj_name2id(viewer.model, mjOBJ_GEOM, kTaskBoxGeomNames[static_cast<std::size_t>(i)]);
    viewer.task_box_body_ids[static_cast<std::size_t>(i)] = body_id;
    viewer.task_box_geom_ids[static_cast<std::size_t>(i)] = geom_id;
    if (body_id < 0) {
      continue;
    }

    const int jnt_adr = viewer.model->body_jntadr[body_id];
    const int qpos_adr = (jnt_adr >= 0) ? viewer.model->jnt_qposadr[jnt_adr] : -1;
    viewer.task_box_qpos_adrs[static_cast<std::size_t>(i)] = qpos_adr;
    if (qpos_adr >= 0) {
      for (int j = 0; j < 7; ++j) {
        viewer.task_box_default_qpos0[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
            viewer.model->qpos0[qpos_adr + j];
      }
    }

    viewer.task_box_default_mass[static_cast<std::size_t>(i)] = viewer.model->body_mass[body_id];
    for (int j = 0; j < 3; ++j) {
      viewer.task_box_default_inertia[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
          viewer.model->body_inertia[3 * body_id + j];
    }
    if (geom_id >= 0) {
      const int rgba_adr = 4 * geom_id;
      for (int j = 0; j < 4; ++j) {
        viewer.task_box_default_rgba[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
            viewer.model->geom_rgba[rgba_adr + j];
      }
    }
  }

  viewer.task_tray_body_id = mj_name2id(viewer.model, mjOBJ_BODY, "desk_tray");
  if (viewer.task_tray_body_id >= 0) {
    const int pos_adr = 3 * viewer.task_tray_body_id;
    const int quat_adr = 4 * viewer.task_tray_body_id;
    for (int i = 0; i < 3; ++i) {
      viewer.task_tray_default_pos[static_cast<std::size_t>(i)] = viewer.model->body_pos[pos_adr + i];
    }
    for (int i = 0; i < 4; ++i) {
      viewer.task_tray_default_quat[static_cast<std::size_t>(i)] = viewer.model->body_quat[quat_adr + i];
    }
  }
}

void RestoreTaskBoxes(ViewerState& viewer) {
  for (int i = 0; i < 2; ++i) {
    const int body_id = viewer.task_box_body_ids[static_cast<std::size_t>(i)];
    const int qpos_adr = viewer.task_box_qpos_adrs[static_cast<std::size_t>(i)];
    if (body_id < 0 || qpos_adr < 0) {
      continue;
    }

    for (int j = 0; j < 7; ++j) {
      viewer.model->qpos0[qpos_adr + j] =
          viewer.task_box_default_qpos0[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
    }
    viewer.model->body_mass[body_id] = viewer.task_box_default_mass[static_cast<std::size_t>(i)];
    for (int j = 0; j < 3; ++j) {
      viewer.model->body_inertia[3 * body_id + j] =
          viewer.task_box_default_inertia[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
    }
    const int geom_id = viewer.task_box_geom_ids[static_cast<std::size_t>(i)];
    if (geom_id >= 0) {
      const int rgba_adr = 4 * geom_id;
      for (int j = 0; j < 4; ++j) {
        viewer.model->geom_rgba[rgba_adr + j] =
            viewer.task_box_default_rgba[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
      }
    }
  }

  if (viewer.task_tray_body_id >= 0) {
    const int pos_adr = 3 * viewer.task_tray_body_id;
    const int quat_adr = 4 * viewer.task_tray_body_id;
    for (int i = 0; i < 3; ++i) {
      viewer.model->body_pos[pos_adr + i] = viewer.task_tray_default_pos[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < 4; ++i) {
      viewer.model->body_quat[quat_adr + i] = viewer.task_tray_default_quat[static_cast<std::size_t>(i)];
    }
  }

  mj_setConst(viewer.model, viewer.data);
}

void RandomizeTaskBoxes(ViewerState& viewer) {
  constexpr mjtNum kXJitter = 0.009;
  constexpr mjtNum kYJitter = 0.009;
  constexpr mjtNum kYawJitter = 4.5 * mjPI / 180.0;
  constexpr mjtNum kMassJitter = 0.3;
  constexpr mjtNum kMinMass = 0.05;
  constexpr mjtNum kMinMassGap = 0.3;
  constexpr mjtNum kTrayXYJitter = 0.009;
  constexpr mjtNum kTrayYawJitter = 4.5 * mjPI / 180.0;

  std::uniform_real_distribution<mjtNum> x_offset_dist(-kXJitter, kXJitter);
  std::uniform_real_distribution<mjtNum> y_offset_dist(-kYJitter, kYJitter);
  std::uniform_real_distribution<mjtNum> yaw_dist(-kYawJitter, kYawJitter);
  std::uniform_real_distribution<mjtNum> mass_offset_dist(-kMassJitter, kMassJitter);
  std::uniform_real_distribution<mjtNum> tray_xy_dist(-kTrayXYJitter, kTrayXYJitter);
  std::uniform_real_distribution<mjtNum> tray_yaw_dist(-kTrayYawJitter, kTrayYawJitter);
  std::array<mjtNum, 2> randomized_mass{};
  for (int attempt = 0; attempt < 64; ++attempt) {
    for (int i = 0; i < 2; ++i) {
      const mjtNum candidate = viewer.task_box_default_mass[static_cast<std::size_t>(i)] + mass_offset_dist(viewer.cloth_rng);
      randomized_mass[static_cast<std::size_t>(i)] = std::max(candidate, kMinMass);
    }
    if (std::abs(randomized_mass[0] - randomized_mass[1]) >= kMinMassGap) {
      break;
    }
    if (attempt == 63) {
      randomized_mass[0] = std::max(viewer.task_box_default_mass[0] + kMassJitter, kMinMass);
      randomized_mass[1] = std::max(viewer.task_box_default_mass[1] - kMassJitter, kMinMass);
    }
  }

  for (int i = 0; i < 2; ++i) {
    const int body_id = viewer.task_box_body_ids[static_cast<std::size_t>(i)];
    const int qpos_adr = viewer.task_box_qpos_adrs[static_cast<std::size_t>(i)];
    if (body_id < 0 || qpos_adr < 0) {
      continue;
    }

    auto qpos0 = viewer.task_box_default_qpos0[static_cast<std::size_t>(i)];
    qpos0[0] += x_offset_dist(viewer.cloth_rng);
    qpos0[1] += y_offset_dist(viewer.cloth_rng);

    const mjtNum yaw = yaw_dist(viewer.cloth_rng);
    qpos0[3] = std::cos(0.5 * yaw);
    qpos0[4] = 0.0;
    qpos0[5] = 0.0;
    qpos0[6] = std::sin(0.5 * yaw);

    for (int j = 0; j < 7; ++j) {
      viewer.model->qpos0[qpos_adr + j] = qpos0[static_cast<std::size_t>(j)];
    }

    const mjtNum mass_scale = randomized_mass[static_cast<std::size_t>(i)] /
                              std::max(viewer.task_box_default_mass[static_cast<std::size_t>(i)], kMinMass);
    viewer.model->body_mass[body_id] = randomized_mass[static_cast<std::size_t>(i)];
    for (int j = 0; j < 3; ++j) {
      viewer.model->body_inertia[3 * body_id + j] =
          viewer.task_box_default_inertia[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * mass_scale;
    }
  }

  if (viewer.task_tray_body_id >= 0) {
    const int pos_adr = 3 * viewer.task_tray_body_id;
    const int quat_adr = 4 * viewer.task_tray_body_id;
    const mjtNum yaw = tray_yaw_dist(viewer.cloth_rng);
    viewer.model->body_pos[pos_adr + 0] = viewer.task_tray_default_pos[0] + tray_xy_dist(viewer.cloth_rng);
    viewer.model->body_pos[pos_adr + 1] = viewer.task_tray_default_pos[1] + tray_xy_dist(viewer.cloth_rng);
    viewer.model->body_pos[pos_adr + 2] = viewer.task_tray_default_pos[2];
    viewer.model->body_quat[quat_adr + 0] = std::cos(0.5 * yaw);
    viewer.model->body_quat[quat_adr + 1] = 0.0;
    viewer.model->body_quat[quat_adr + 2] = 0.0;
    viewer.model->body_quat[quat_adr + 3] = std::sin(0.5 * yaw);
  }

  mj_setConst(viewer.model, viewer.data);
}

void RandomizeClothRootPose(ViewerState& viewer) {
  if (viewer.cloth_body_id < 0) {
    return;
  }

  std::uniform_real_distribution<mjtNum> x_offset_dist(-0.06, 0.06);
  std::uniform_real_distribution<mjtNum> y_offset_dist(-0.30, 0.30);
  std::uniform_real_distribution<mjtNum> roll_dist(-mjPI, mjPI);
  std::uniform_real_distribution<mjtNum> pitch_dist(-mjPI, mjPI);
  std::uniform_real_distribution<mjtNum> yaw_dist(-mjPI, mjPI);
  const mjtNum x = viewer.cloth_default_pos[0] + x_offset_dist(viewer.cloth_rng);
  const mjtNum y = viewer.cloth_default_pos[1] + y_offset_dist(viewer.cloth_rng);
  const mjtNum roll = roll_dist(viewer.cloth_rng);
  const mjtNum pitch = pitch_dist(viewer.cloth_rng);
  const mjtNum yaw = yaw_dist(viewer.cloth_rng);
  SetClothRootPose(viewer, x, y, roll, pitch, yaw);
}

void SettleTaskCloth(ViewerState& viewer) {
  constexpr double kTaskClothSettleDuration = 0.5;
  const mjtNum timestep = std::max<mjtNum>(viewer.model->opt.timestep, 1e-6);
  const int settle_steps = std::max(1, static_cast<int>(kTaskClothSettleDuration / timestep));

  mju_zero(viewer.data->qfrc_applied, viewer.model->nv);
  mju_zero(viewer.data->xfrc_applied, 6 * viewer.model->nbody);
  mju_zero(viewer.data->qacc_warmstart, viewer.model->nv);
  if (viewer.model->nu > 0) {
    mju_zero(viewer.data->ctrl, viewer.model->nu);
  }
  if (viewer.model->na > 0) {
    mju_zero(viewer.data->act, viewer.model->na);
  }

  mj_forward(viewer.model, viewer.data);
  for (int i = 0; i < settle_steps; ++i) {
    mj_step(viewer.model, viewer.data);
  }
}

void ResetSimulationAndController(GLFWwindow* window) {
  if (!g_viewer) {
    return;
  }

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);

  RestoreDefaultClothRootPose(*g_viewer);
  RestoreDefaultClothColor(*g_viewer);
  RestoreTaskBoxes(*g_viewer);
  mj_resetData(g_viewer->model, g_viewer->data);
  mj_forward(g_viewer->model, g_viewer->data);

  g_viewer->paused = false;
  g_viewer->request_single_step = false;
  g_viewer->realtime_scale = 1.0;
  g_viewer->slow_motion = false;
  g_viewer->joint_page = 0;
  g_viewer->actuator_page = 0;
  g_viewer->category_pages.fill(0);
  CancelGraphEdit(*g_viewer);
  ClearJointHistory(*g_viewer);
  mjv_defaultPerturb(&g_viewer->pert);
  if (g_viewer->controller) {
    g_viewer->controller->Reset();
  }

  mjv_updateScene(g_viewer->model, g_viewer->data, &g_viewer->opt, &g_viewer->pert, &g_viewer->cam, mjCAT_ALL,
                  &g_viewer->scn);
  (void)width;
  (void)height;
}

void ResetSimulationAndControllerToMode2(GLFWwindow* window) {
  if (!g_viewer) {
    return;
  }

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);

  RandomizeClothRootPose(*g_viewer);
  RandomizeTaskBoxes(*g_viewer);
  mj_resetData(g_viewer->model, g_viewer->data);
  SettleTaskCloth(*g_viewer);
  if (g_viewer->controller) {
    g_viewer->controller->ApplyWorkReadyPose();
  }
  g_viewer->data->time = 0.0;
  mj_forward(g_viewer->model, g_viewer->data);

  g_viewer->paused = false;
  g_viewer->request_single_step = false;
  g_viewer->realtime_scale = 1.0;
  g_viewer->slow_motion = false;
  g_viewer->joint_page = 0;
  g_viewer->actuator_page = 0;
  g_viewer->category_pages.fill(0);
  CancelGraphEdit(*g_viewer);
  ClearJointHistory(*g_viewer);
  mjv_defaultPerturb(&g_viewer->pert);
  if (g_viewer->controller) {
    g_viewer->controller->ResetToMode2();
  }

  mjv_updateScene(g_viewer->model, g_viewer->data, &g_viewer->opt, &g_viewer->pert, &g_viewer->cam, mjCAT_ALL,
                  &g_viewer->scn);
  (void)width;
  (void)height;
}

void CycleFrameMode() {
  if (!g_viewer) {
    return;
  }

  switch (g_viewer->opt.frame) {
    case mjFRAME_NONE:
      g_viewer->opt.frame = mjFRAME_BODY;
      break;
    case mjFRAME_BODY:
      g_viewer->opt.frame = mjFRAME_GEOM;
      break;
    case mjFRAME_GEOM:
      g_viewer->opt.frame = mjFRAME_SITE;
      break;
    case mjFRAME_SITE:
      g_viewer->opt.frame = mjFRAME_WORLD;
      break;
    default:
      g_viewer->opt.frame = mjFRAME_NONE;
      break;
  }
}

void ApplyCameraMode() {
  if (!g_viewer) {
    return;
  }

  if (g_viewer->camera_mode <= 0 || g_viewer->model->ncam <= 0) {
    g_viewer->cam.type = mjCAMERA_FREE;
    g_viewer->cam.fixedcamid = -1;
    return;
  }

  const int fixed_id = g_viewer->camera_mode - 1;
  if (fixed_id >= 0 && fixed_id < g_viewer->model->ncam) {
    g_viewer->cam.type = mjCAMERA_FIXED;
    g_viewer->cam.fixedcamid = fixed_id;
  } else {
    g_viewer->camera_mode = 0;
    g_viewer->cam.type = mjCAMERA_FREE;
    g_viewer->cam.fixedcamid = -1;
  }
}

void CycleCameraMode() {
  if (!g_viewer) {
    return;
  }

  const int total_modes = g_viewer->model->ncam + 1;
  g_viewer->camera_mode = (g_viewer->camera_mode + 1) % std::max(1, total_modes);
  ApplyCameraMode();
}

bool HandleUiPress(GLFWwindow* window, double xpos, double ypos) {
  if (!g_viewer) {
    return false;
  }

  const DesktopUi ui = CurrentDesktopUi(window);
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  const mjrRect vp = {0, 0, width, height};

  if (g_viewer->editing_graph_field != kGraphEditNone) {
    bool keep_editing = false;
    if (g_viewer->editing_graph_window >= 0 && g_viewer->editing_graph_window < static_cast<int>(ui.graph_windows.size())) {
      const GraphWindowUi& editing_ui = ui.graph_windows[static_cast<std::size_t>(g_viewer->editing_graph_window)];
      if ((g_viewer->editing_graph_field == kGraphEditXSeconds && PointInRect(vp, editing_ui.x_value_button, xpos, ypos)) ||
          (g_viewer->editing_graph_field == kGraphEditYLimit && PointInRect(vp, editing_ui.y_value_button, xpos, ypos))) {
        keep_editing = true;
      }
    }
    if (keep_editing) {
      g_viewer->ui_capture = true;
      return true;
    }
    CommitGraphEdit(*g_viewer);
  }

  for (auto it = g_viewer->graph_window_order.rbegin(); it != g_viewer->graph_window_order.rend(); ++it) {
    const int graph_index = *it;
    if (graph_index < 0 || graph_index >= static_cast<int>(ui.graph_windows.size())) {
      continue;
    }
    const GraphWindowUi& graph = ui.graph_windows[static_cast<std::size_t>(graph_index)];
    if (!graph.visible || !PointInRect(vp, graph.rect, xpos, ypos)) {
      continue;
    }

    FocusGraphWindow(*g_viewer, graph_index);

    if (PointInRect(vp, graph.close_button, xpos, ypos)) {
      CloseGraphWindow(*g_viewer, graph_index);
      g_viewer->ui_capture = true;
      return true;
    }
    if (PointInRect(vp, graph.minimize_button, xpos, ypos)) {
      g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].minimized =
          !g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].minimized;
      g_viewer->ui_capture = true;
      return true;
    }

    if (!graph.minimized) {
      if (PointInRect(vp, graph.selector_toggle_button, xpos, ypos)) {
        g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].show_selector =
            !g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].show_selector;
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.y_mode_button, xpos, ypos)) {
        g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].y_auto =
            !g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].y_auto;
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.y_value_button, xpos, ypos)) {
        BeginGraphEdit(*g_viewer, graph_index, kGraphEditYLimit);
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.y_minus_button, xpos, ypos)) {
        GraphWindowState& state = g_viewer->graph_windows[static_cast<std::size_t>(graph_index)];
        if (!state.y_auto) {
          state.y_manual_limit = StepPresetValue(state.y_manual_limit, kGraphYLimitPresets, -1);
        }
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.y_plus_button, xpos, ypos)) {
        GraphWindowState& state = g_viewer->graph_windows[static_cast<std::size_t>(graph_index)];
        if (!state.y_auto) {
          state.y_manual_limit = StepPresetValue(state.y_manual_limit, kGraphYLimitPresets, +1);
        }
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.x_minus_button, xpos, ypos)) {
        GraphWindowState& state = g_viewer->graph_windows[static_cast<std::size_t>(graph_index)];
        state.x_seconds = StepPresetValue(state.x_seconds, kGraphXSecondsPresets, -1);
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.x_value_button, xpos, ypos)) {
        BeginGraphEdit(*g_viewer, graph_index, kGraphEditXSeconds);
        g_viewer->ui_capture = true;
        return true;
      }
      if (PointInRect(vp, graph.x_plus_button, xpos, ypos)) {
        GraphWindowState& state = g_viewer->graph_windows[static_cast<std::size_t>(graph_index)];
        state.x_seconds = StepPresetValue(state.x_seconds, kGraphXSecondsPresets, +1);
        g_viewer->ui_capture = true;
        return true;
      }

      if (graph.show_selector && PointInRect(vp, graph.prev_button, xpos, ypos)) {
        g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].joint_page =
            std::max(0, g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].joint_page - 1);
        g_viewer->ui_capture = true;
        return true;
      }
      if (graph.show_selector && PointInRect(vp, graph.next_button, xpos, ypos)) {
        g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].joint_page =
            std::min(graph.total_pages - 1, g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].joint_page + 1);
        g_viewer->ui_capture = true;
        return true;
      }
      if (graph.show_selector) {
        for (std::size_t i = 0; i < graph.joint_rows.size() && i < graph.visible_joint_ids.size(); ++i) {
          if (!PointInRect(vp, graph.joint_rows[i], xpos, ypos)) {
            continue;
          }
          g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].selected_joint = graph.visible_joint_ids[i];
          g_viewer->ui_capture = true;
          return true;
        }
      }
    }

    if (PointInRect(vp, graph.header_rect, xpos, ypos)) {
      g_viewer->drag_graph_window = graph_index;
      g_viewer->drag_offset_x = xpos - g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].left;
      g_viewer->drag_offset_y = ypos - g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].top;
      g_viewer->graph_windows[static_cast<std::size_t>(graph_index)].user_moved = true;
      g_viewer->ui_capture = true;
      return true;
    }

    g_viewer->ui_capture = true;
    return true;
  }

  for (int category = 0; category < kCategoryCount; ++category) {
    if (PointInRect(vp, ui.dock.buttons[category], xpos, ypos)) {
      OpenWindow(*g_viewer, category);
      g_viewer->ui_capture = true;
      return true;
    }
  }

  for (auto it = g_viewer->window_order.rbegin(); it != g_viewer->window_order.rend(); ++it) {
    const int category = *it;
    const FloatingWindowUi& win = ui.windows[category];
    if (!win.visible || !PointInRect(vp, win.rect, xpos, ypos)) {
      continue;
    }

    FocusWindow(*g_viewer, category);

    if (PointInRect(vp, win.close_button, xpos, ypos)) {
      CloseWindow(*g_viewer, category);
      g_viewer->ui_capture = true;
      return true;
    }

    if (PointInRect(vp, win.minimize_button, xpos, ypos)) {
      g_viewer->floating_windows[category].minimized = !g_viewer->floating_windows[category].minimized;
      g_viewer->ui_capture = true;
      return true;
    }

    if (!win.minimized) {
      if (win.show_status_actions && PointInRect(vp, win.pause_button, xpos, ypos)) {
        g_viewer->paused = !g_viewer->paused;
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_status_actions && PointInRect(vp, win.step_button, xpos, ypos)) {
        g_viewer->request_single_step = true;
        g_viewer->paused = true;
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_status_actions && PointInRect(vp, win.task_reset_button, xpos, ypos)) {
        ResetSimulationAndControllerToMode2(window);
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_status_actions && PointInRect(vp, win.reset_button, xpos, ypos)) {
        ResetSimulationAndController(window);
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_favorites_button && PointInRect(vp, win.favorite_filter_button, xpos, ypos)) {
        g_viewer->favorites_only[category] = !g_viewer->favorites_only[category];
        g_viewer->category_pages[category] = 0;
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_graph_button && PointInRect(vp, win.graph_button, xpos, ypos)) {
        const int initial_joint = (!win.entries.empty() && win.entries.front().item.item_index >= 0) ? win.entries.front().item.item_index
                                                                                                      : DefaultGraphJoint(*g_viewer);
        OpenGraphWindow(*g_viewer, initial_joint);
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_page_controls && PointInRect(vp, win.prev_button, xpos, ypos)) {
        g_viewer->category_pages[category] = std::max(0, g_viewer->category_pages[category] - 1);
        g_viewer->ui_capture = true;
        return true;
      }
      if (win.show_page_controls && PointInRect(vp, win.next_button, xpos, ypos)) {
        g_viewer->category_pages[category] = std::min(win.total_pages - 1, g_viewer->category_pages[category] + 1);
        g_viewer->ui_capture = true;
        return true;
      }

      for (const DrawerEntryUi& entry : win.entries) {
        if (entry.item.can_favorite && PointInRect(vp, entry.favorite_button, xpos, ypos)) {
          ToggleFavorite(*g_viewer, entry.item.source_category, entry.item.item_index);
          g_viewer->ui_capture = true;
          return true;
        }
        if (PointInRect(vp, entry.rect, xpos, ypos)) {
          g_viewer->ui_capture = true;
          return true;
        }
      }
    }

    if (PointInRect(vp, win.header_rect, xpos, ypos)) {
      g_viewer->drag_window = category;
      g_viewer->drag_offset_x = xpos - g_viewer->floating_windows[category].left;
      g_viewer->drag_offset_y = ypos - g_viewer->floating_windows[category].top;
      g_viewer->floating_windows[category].user_moved = true;
      g_viewer->ui_capture = true;
      return true;
    }

    g_viewer->ui_capture = true;
    return true;
  }

  return false;
}

void HandleUiRelease() {
  if (!g_viewer) {
    return;
  }
  g_viewer->drag_window = -1;
  g_viewer->drag_graph_window = -1;
  g_viewer->ui_capture = false;
}

void HandleUiDrag(GLFWwindow* window, double xpos, double ypos) {
  if (!g_viewer) {
    return;
  }

  const DesktopUi ui = CurrentDesktopUi(window);
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  const mjrRect vp = {0, 0, width, height};

  if (g_viewer->drag_graph_window >= 0 &&
      g_viewer->drag_graph_window < static_cast<int>(g_viewer->graph_windows.size()) &&
      g_viewer->drag_graph_window < static_cast<int>(ui.graph_windows.size())) {
    GraphWindowState& state = g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->drag_graph_window)];
    const GraphWindowUi& win = ui.graph_windows[static_cast<std::size_t>(g_viewer->drag_graph_window)];
    state.left = static_cast<int>(std::lround(xpos - g_viewer->drag_offset_x));
    state.top = static_cast<int>(std::lround(ypos - g_viewer->drag_offset_y));
    state.user_moved = true;
    ClampGraphWindow(state, vp, win.rect.width, win.rect.height);
    return;
  }

  if (g_viewer->drag_window < 0) {
    return;
  }

  FloatingWindowState& state = g_viewer->floating_windows[g_viewer->drag_window];
  const FloatingWindowUi& win = ui.windows[g_viewer->drag_window];
  state.left = static_cast<int>(std::lround(xpos - g_viewer->drag_offset_x));
  state.top = static_cast<int>(std::lround(ypos - g_viewer->drag_offset_y));
  state.user_moved = true;
  ClampFloatingWindow(state, vp, win.rect.width, win.rect.height);
}

void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
  (void)scancode;
  (void)mods;
  if (!g_viewer || act != GLFW_PRESS) {
    return;
  }

  if (HandleGraphEditKey(*g_viewer, key)) {
    return;
  }

  switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;
    case GLFW_KEY_BACKSPACE:
    case GLFW_KEY_R:
      ResetSimulationAndController(window);
      break;
    case GLFW_KEY_SPACE:
      g_viewer->paused = !g_viewer->paused;
      break;
    case GLFW_KEY_RIGHT:
      g_viewer->request_single_step = true;
      g_viewer->paused = true;
      break;
    case GLFW_KEY_H:
      g_viewer->show_help = !g_viewer->show_help;
      g_viewer->category_pages[kCategoryKeys] = 0;
      OpenWindow(*g_viewer, kCategoryKeys);
      break;
    case GLFW_KEY_I:
      if (g_viewer->active_category == kCategoryJoints) {
        OpenWindow(*g_viewer, kCategoryActuators);
      } else {
        OpenWindow(*g_viewer, kCategoryJoints);
      }
      break;
    case GLFW_KEY_F:
      CycleFrameMode();
      break;
    case GLFW_KEY_C:
      g_viewer->opt.flags[mjVIS_CONTACTPOINT] = !g_viewer->opt.flags[mjVIS_CONTACTPOINT];
      g_viewer->opt.flags[mjVIS_CONTACTFORCE] = !g_viewer->opt.flags[mjVIS_CONTACTFORCE];
      break;
    case GLFW_KEY_J:
      g_viewer->opt.flags[mjVIS_JOINT] = !g_viewer->opt.flags[mjVIS_JOINT];
      break;
    case GLFW_KEY_A:
      g_viewer->opt.flags[mjVIS_ACTUATOR] = !g_viewer->opt.flags[mjVIS_ACTUATOR];
      break;
    case GLFW_KEY_W:
      g_viewer->scn.flags[mjRND_WIREFRAME] = !g_viewer->scn.flags[mjRND_WIREFRAME];
      break;
    case GLFW_KEY_S:
      g_viewer->scn.flags[mjRND_SHADOW] = !g_viewer->scn.flags[mjRND_SHADOW];
      break;
    case GLFW_KEY_T:
      g_viewer->slow_motion = !g_viewer->slow_motion;
      break;
    case GLFW_KEY_MINUS:
      g_viewer->realtime_scale = std::max(0.1, g_viewer->realtime_scale * 0.8);
      break;
    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD:
      g_viewer->realtime_scale = std::min(4.0, g_viewer->realtime_scale * 1.25);
      break;
    case GLFW_KEY_KP_SUBTRACT:
      g_viewer->realtime_scale = std::max(0.1, g_viewer->realtime_scale * 0.8);
      break;
    case GLFW_KEY_TAB:
      CycleCameraMode();
      break;
    case GLFW_KEY_0:
      g_viewer->camera_mode = 0;
      ApplyCameraMode();
      break;
    case GLFW_KEY_PAGE_UP:
      if (g_viewer->focused_graph_window >= 0 &&
          g_viewer->focused_graph_window < static_cast<int>(g_viewer->graph_windows.size()) &&
          g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].visible) {
        g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page =
            std::max(0, g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page - 1);
      } else if (g_viewer->floating_windows[g_viewer->active_category].visible) {
        g_viewer->category_pages[g_viewer->active_category] =
            std::max(0, g_viewer->category_pages[g_viewer->active_category] - 1);
      }
      break;
    case GLFW_KEY_PAGE_DOWN: {
      const DesktopUi ui = CurrentDesktopUi(window);
      if (g_viewer->focused_graph_window >= 0 &&
          g_viewer->focused_graph_window < static_cast<int>(g_viewer->graph_windows.size()) &&
          g_viewer->focused_graph_window < static_cast<int>(ui.graph_windows.size()) &&
          ui.graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].visible) {
        g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page =
            std::min(ui.graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].total_pages - 1,
                     g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page + 1);
      } else {
        const int category = g_viewer->floating_windows[g_viewer->active_category].visible ? g_viewer->active_category
                                                                                            : TopVisibleWindowCategory(*g_viewer);
        if (ui.windows[category].visible) {
          g_viewer->category_pages[category] =
              std::min(ui.windows[category].total_pages - 1, g_viewer->category_pages[category] + 1);
        }
      }
      break;
    }
    case GLFW_KEY_HOME:
      if (g_viewer->focused_graph_window >= 0 &&
          g_viewer->focused_graph_window < static_cast<int>(g_viewer->graph_windows.size()) &&
          g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].visible) {
        g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page = 0;
      } else if (g_viewer->floating_windows[g_viewer->active_category].visible) {
        g_viewer->category_pages[g_viewer->active_category] = 0;
      }
      break;
    case GLFW_KEY_END: {
      const DesktopUi ui = CurrentDesktopUi(window);
      if (g_viewer->focused_graph_window >= 0 &&
          g_viewer->focused_graph_window < static_cast<int>(g_viewer->graph_windows.size()) &&
          g_viewer->focused_graph_window < static_cast<int>(ui.graph_windows.size()) &&
          ui.graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].visible) {
        g_viewer->graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].joint_page =
            std::max(0, ui.graph_windows[static_cast<std::size_t>(g_viewer->focused_graph_window)].total_pages - 1);
      } else {
        const int category = g_viewer->floating_windows[g_viewer->active_category].visible ? g_viewer->active_category
                                                                                            : TopVisibleWindowCategory(*g_viewer);
        if (ui.windows[category].visible) {
          g_viewer->category_pages[category] = std::max(0, ui.windows[category].total_pages - 1);
        }
      }
      break;
    }
    default:
      break;
  }
}

void StartPerturb(GLFWwindow* window, int button) {
  if (!g_viewer) {
    return;
  }

  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  if (width <= 0 || height <= 0) {
    return;
  }

  mjtNum selpnt[3] = {0, 0, 0};
  int geomid[1] = {-1};
  int flexid[1] = {-1};
  int skinid[1] = {-1};

  const mjtNum relx = static_cast<mjtNum>(g_viewer->lastx / std::max(1, width));
  const mjtNum rely = static_cast<mjtNum>(1.0 - g_viewer->lasty / std::max(1, height));

  const int bodyid = mjv_select(g_viewer->model, g_viewer->data, &g_viewer->opt,
                                static_cast<mjtNum>(width) / std::max(1, height), relx, rely, &g_viewer->scn,
                                selpnt, geomid, flexid, skinid);

  if (bodyid <= 0) {
    ClearPerturb();
    g_viewer->pert.select = 0;
    return;
  }

  g_viewer->pert.select = bodyid;
  g_viewer->pert.flexselect = flexid[0];
  g_viewer->pert.skinselect = skinid[0];
  g_viewer->pert.active = (button == GLFW_MOUSE_BUTTON_RIGHT) ? mjPERT_ROTATE : mjPERT_TRANSLATE;
  g_viewer->pert.active2 = 0;
  mjv_initPerturb(g_viewer->model, g_viewer->data, &g_viewer->scn, &g_viewer->pert);

  // Force/torque application point: use the mouse-picked 3D point in the selected body's frame.
  // mjvPerturb::localpos is expressed in body coordinates, not in the inertial/COM frame.
  mju_copy3(g_viewer->pert.refselpos, selpnt);
  mjtNum diff_world[3] = {selpnt[0] - g_viewer->data->xpos[3 * bodyid + 0],
                          selpnt[1] - g_viewer->data->xpos[3 * bodyid + 1],
                          selpnt[2] - g_viewer->data->xpos[3 * bodyid + 2]};
  const mjtNum* R = g_viewer->data->xmat + 9 * bodyid;
  g_viewer->pert.localpos[0] = R[0] * diff_world[0] + R[3] * diff_world[1] + R[6] * diff_world[2];
  g_viewer->pert.localpos[1] = R[1] * diff_world[0] + R[4] * diff_world[1] + R[7] * diff_world[2];
  g_viewer->pert.localpos[2] = R[2] * diff_world[0] + R[5] * diff_world[1] + R[8] * diff_world[2];
}

void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
  (void)mods;
  if (!g_viewer) {
    return;
  }

  glfwGetCursorPos(window, &g_viewer->lastx, &g_viewer->lasty);

  if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS && HandleUiPress(window, g_viewer->lastx, g_viewer->lasty)) {
    g_viewer->button_left = true;
    g_viewer->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    g_viewer->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    return;
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_RELEASE && g_viewer->ui_capture) {
    HandleUiRelease();
    g_viewer->button_left = false;
    g_viewer->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    g_viewer->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    return;
  }

  g_viewer->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  g_viewer->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  g_viewer->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  if (act == GLFW_PRESS && IsCtrlPressed(window) &&
      (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT)) {
    StartPerturb(window, button);
  }

  if (act == GLFW_RELEASE &&
      (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE)) {
    ClearPerturb();
  }
}

void CursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  if (g_viewer && g_viewer->ui_capture) {
    g_viewer->lastx = xpos;
    g_viewer->lasty = ypos;
    HandleUiDrag(window, xpos, ypos);
    return;
  }

  if (!g_viewer || (!g_viewer->button_left && !g_viewer->button_middle && !g_viewer->button_right)) {
    return;
  }

  const double dx = xpos - g_viewer->lastx;
  const double dy = ypos - g_viewer->lasty;
  g_viewer->lastx = xpos;
  g_viewer->lasty = ypos;

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);

  const bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  const bool mod_ctrl = IsCtrlPressed(window);

  if (mod_ctrl && g_viewer->pert.select > 0 && g_viewer->pert.active) {
    mjtMouse action;
    if (g_viewer->pert.active == mjPERT_ROTATE) {
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }

    mjv_movePerturb(g_viewer->model, g_viewer->data, action, dx / std::max(1, height), dy / std::max(1, height),
                    &g_viewer->scn, &g_viewer->pert);
    return;
  }

  mjtMouse action;
  if (g_viewer->button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (g_viewer->button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(g_viewer->model, action, dx / std::max(1, height), dy / std::max(1, height), &g_viewer->scn,
                 &g_viewer->cam);
}

void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  (void)window;
  (void)xoffset;
  if (!g_viewer) {
    return;
  }
  mjv_moveCamera(g_viewer->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &g_viewer->scn, &g_viewer->cam);
}

void DrawHud(const ViewerState& viewer, const DaruV4MuJoCoController& controller, const mjrRect& viewport,
             int step_count) {
  (void)controller;
  EnsureUiIconsLoaded(*g_viewer);
  g_viewer->step_count = step_count;
  UpdateJointHistory(*g_viewer);
  const DesktopUi ui = ComputeDesktopUi(*g_viewer, viewport);
  DrawDesktopUi(viewer, ui);
}

void RenderFixedCameraRGB(ViewerState& viewer,
                          const char* cam_name,
                          int width,
                          int height,
                          std::vector<uint8_t>& rgb_out) {
  const int cam_id = mj_name2id(viewer.model, mjOBJ_CAMERA, cam_name);
  if (cam_id < 0) {
    rgb_out.clear();
    return;
  }

  rgb_out.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = cam_id;

  mjrRect viewport{0, 0, width, height};

  mjv_updateScene(viewer.model, viewer.data, &viewer.opt, &viewer.pert, &cam, mjCAT_ALL,
                  &viewer.offscreen_scn);
  mjr_setBuffer(mjFB_OFFSCREEN, &viewer.con);
  mjr_render(viewport, &viewer.offscreen_scn, &viewer.con);
  mjr_readPixels(rgb_out.data(), nullptr, viewport, &viewer.con);

  const int row_bytes = width * 3;
  std::vector<uint8_t> flipped(rgb_out.size());
  for (int y = 0; y < height; ++y) {
    const uint8_t* src = rgb_out.data() + (height - 1 - y) * row_bytes;
    uint8_t* dst = flipped.data() + y * row_bytes;
    std::memcpy(dst, src, row_bytes);
  }
  rgb_out.swap(flipped);
  mjr_setBuffer(mjFB_WINDOW, &viewer.con);
}

void PublishStereoFromGui(ViewerState& viewer,
                          DaruV4MuJoCoRosBridge& ros_bridge,
                          int width,
                          int height) {
  std::vector<uint8_t> left_rgb;
  std::vector<uint8_t> right_rgb;
  std::vector<uint8_t> left_wrist_rgb;
  std::vector<uint8_t> right_wrist_rgb;
  const rclcpp::Time stamp = rclcpp::Clock().now();

  RenderFixedCameraRGB(viewer, "L_front_cam", width, height, left_rgb);
  RenderFixedCameraRGB(viewer, "R_front_cam", width, height, right_rgb);
  RenderFixedCameraRGB(viewer, "L_wrist_cam", width, height, left_wrist_rgb);
  RenderFixedCameraRGB(viewer, "R_wrist_cam", width, height, right_wrist_rgb);

  if (left_rgb.size() != static_cast<std::size_t>(width * height * 3) ||
      right_rgb.size() != static_cast<std::size_t>(width * height * 3)) {
  } else {
    ros_bridge.PublishStereoFrames(left_rgb, right_rgb, width, height, stamp);
  }

  if (left_wrist_rgb.size() != static_cast<std::size_t>(width * height * 3) ||
      right_wrist_rgb.size() != static_cast<std::size_t>(width * height * 3)) {
    return;
  }

  ros_bridge.PublishWristFrames(left_wrist_rgb, right_wrist_rgb, width, height, stamp);
}

}  // namespace

int RunTeleopGui(mjModel* model,
                 mjData* data,
                 DaruV4MuJoCoController& controller,
                 DaruV4MuJoCoRosBridge& ros_bridge,
                 int max_steps,
                 const std::function<void()>& step_hook) {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 5;
  }

  glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
  GLFWmonitor* primary_monitor = glfwGetPrimaryMonitor();
  int window_width = 1480;
  int window_height = 920;
  if (primary_monitor) {
    const GLFWvidmode* mode = glfwGetVideoMode(primary_monitor);
    if (mode) {
      window_width = std::max(1280, mode->width * 94 / 100);
      window_height = std::max(800, mode->height * 94 / 100);
    }
  }
  GLFWwindow* window = glfwCreateWindow(window_width, window_height, "DARU MuJoCo Control Room", nullptr, nullptr);
  if (!window) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 6;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);
  glfwMaximizeWindow(window);

  ViewerState viewer;
  viewer.model = model;
  viewer.data = data;
  viewer.controller = &controller;
  viewer.favorite_joints.assign(static_cast<std::size_t>(model->njnt), 0);
  viewer.favorite_actuators.assign(static_cast<std::size_t>(model->nu), 0);
  viewer.joint_history.resize(static_cast<std::size_t>(model->njnt));
  ClearJointHistory(viewer);
  viewer.floating_windows[kCategoryStatus].visible = true;
  viewer.floating_windows[kCategoryJoints].visible = true;
  FocusWindow(viewer, kCategoryStatus);
  FocusWindow(viewer, kCategoryJoints);
  mjv_defaultCamera(&viewer.cam);
  ApplyDefaultFreeCameraPose(viewer.cam);
  mjv_defaultOption(&viewer.opt);
  mjv_defaultScene(&viewer.scn);
  mjv_defaultScene(&viewer.offscreen_scn);
  mjr_defaultContext(&viewer.con);
  mjv_defaultPerturb(&viewer.pert);

  viewer.opt.flags[mjVIS_CONTACTPOINT] = 0;
  viewer.opt.flags[mjVIS_CONTACTFORCE] = 0;
  viewer.opt.flags[mjVIS_JOINT] = 0;
  viewer.opt.flags[mjVIS_ACTUATOR] = 0;
  viewer.scn.flags[mjRND_SHADOW] = 1;
  viewer.scn.flags[mjRND_WIREFRAME] = 0;
  viewer.offscreen_scn.flags[mjRND_SHADOW] = 1;
  viewer.offscreen_scn.flags[mjRND_WIREFRAME] = 0;

  mjv_makeScene(model, &viewer.scn, 4000);
  mjv_makeScene(model, &viewer.offscreen_scn, 4000);
  CacheClothRootPose(viewer);
  CacheTaskBoxes(viewer);
  int init_width = 0;
  int init_height = 0;
  glfwGetFramebufferSize(window, &init_width, &init_height);
  UpdateUiScale(viewer, init_width, init_height);
  g_viewer = &viewer;
  EnsureUiIconsLoaded(viewer);
  ShowStartupSplash(window, viewer, controller);

  glfwSetKeyCallback(window, KeyboardCallback);
  glfwSetMouseButtonCallback(window, MouseButtonCallback);
  glfwSetCursorPosCallback(window, CursorPosCallback);
  glfwSetScrollCallback(window, ScrollCallback);

  int step = 0;
  const double dt = model->opt.timestep;
  const int max_substeps_per_frame = 256;
  double wall_t0 = glfwGetTime();
  double sim_t0 = data->time;

  while (!glfwWindowShouldClose(window)) {
    const double scale = viewer.slow_motion ? 0.25 * viewer.realtime_scale : viewer.realtime_scale;

    int substeps = 0;
    if (viewer.request_single_step) {
      controller.Step();
      mju_zero(data->xfrc_applied, 6 * model->nbody);
      if (viewer.pert.select > 0 && viewer.pert.active) {
        mjv_applyPerturbForce(model, data, &viewer.pert);
      }
      mj_step(model, data);
      ++substeps;
      ++step;
      viewer.request_single_step = false;
    } else if (!viewer.paused) {
      const double wall_now = glfwGetTime();
      const double wall_elapsed = wall_now - wall_t0;
      const double target_sim_time = sim_t0 + wall_elapsed * scale;

      while (data->time + 0.5 * dt < target_sim_time && substeps < max_substeps_per_frame) {
        controller.Step();
        mju_zero(data->xfrc_applied, 6 * model->nbody);
        if (viewer.pert.select > 0 && viewer.pert.active) {
          mjv_applyPerturbForce(model, data, &viewer.pert);
        }
        mj_step(model, data);
        ++substeps;
        ++step;
        if (max_steps > 0 && step >= max_steps) {
          break;
        }
      }
    } else {
      wall_t0 = glfwGetTime();
      sim_t0 = data->time;
    }

    if (substeps > 0 && !viewer.paused) {
      wall_t0 = glfwGetTime();
      sim_t0 = data->time;
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    UpdateUiScale(viewer, width, height);
    mjrRect viewport = {0, 0, width, height};

    mjv_updateScene(model, data, &viewer.opt, &viewer.pert, &viewer.cam, mjCAT_ALL, &viewer.scn);
    mjr_render(viewport, &viewer.scn, &viewer.con);
    DrawHud(viewer, controller, viewport, step);

    static double last_pub_time = 0.0;
    const double now = glfwGetTime();
    if (now - last_pub_time > 0.033) {   // 30 Hz
      PublishStereoFromGui(viewer, ros_bridge, 340, 180);
      last_pub_time = now;
    }

    glfwSwapBuffers(window);
    glfwPollEvents();

    if (step_hook) {
      step_hook();
    }

    if (ros_bridge.ConsumeTaskResetRequest()) {
      ResetSimulationAndControllerToMode2(window);
    }

    if (max_steps > 0 && step >= max_steps) {
      break;
    }

    if (substeps == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  g_viewer = nullptr;
  FreeUiIcons(viewer);
  mjv_freeScene(&viewer.offscreen_scn);
  mjv_freeScene(&viewer.scn);
  mjr_freeContext(&viewer.con);
  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}

}  // namespace daru_mj
