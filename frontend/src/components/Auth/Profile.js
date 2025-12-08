import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../components/Auth/AuthProvider";
import { useHistory } from "@docusaurus/router";
import BrowserOnly from "@docusaurus/BrowserOnly";

const ProfileContent = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();

  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({
    technical_background: "",
    hardware_access: "",
    learning_goals: "",
  });
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (!loading && !user) {
      history.push("/");
    }
  }, [user, loading, history]);

  useEffect(() => {
    if (user) {
      setProfileData({
        technical_background: user.technical_background || "",
        hardware_access: user.hardware_access || "",
        learning_goals: user.learning_goals || "",
      });
    }
  }, [user]);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setProfileData((prev) => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    const result = await updateProfile(profileData);
    if (result.success) {
      setMessage("Profile updated successfully!");
      setTimeout(() => setIsEditing(false), 1000);
    } else {
      setMessage(result.message || "Failed to update");
    }
  };

  // --- INLINE STYLES OBJECTS (Guaranteed to work) ---
  const styles = {
    wrapper: {
      display: "flex",
      justifyContent: "center",
      padding: "4rem 1rem",
      backgroundColor: "#151e29", // Dark background for page
      minHeight: "85vh",
      fontFamily:
        '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
    },
    card: {
      backgroundColor: "#1E2A38", // Dark Blue Card
      padding: "2.5rem",
      borderRadius: "16px",
      width: "100%",
      maxWidth: "600px",
      boxShadow: "0 10px 40px rgba(0,0,0,0.5)",
      border: "1px solid #2C3E50",
      color: "#ffffff",
    },
    header: {
      display: "flex",
      alignItems: "center",
      gap: "20px",
      marginBottom: "20px",
    },
    avatar: {
      width: "70px",
      height: "70px",
      background: "linear-gradient(135deg, #2ECC71, #27ae60)",
      borderRadius: "50%",
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      fontSize: "32px",
      fontWeight: "bold",
      color: "#1E2A38",
    },
    h2: { margin: 0, color: "white", fontSize: "24px" },
    email: { color: "#94a3b8", margin: "4px 0 8px 0", fontSize: "14px" },
    badge: {
      backgroundColor: "#8b5cf6",
      color: "white",
      padding: "4px 10px",
      borderRadius: "12px",
      fontSize: "11px",
      fontWeight: "bold",
      textTransform: "uppercase",
    },
    divider: {
      border: 0,
      height: "1px",
      background: "#2C3E50",
      margin: "20px 0",
    },
    label: {
      display: "block",
      color: "#94a3b8",
      fontSize: "12px",
      textTransform: "uppercase",
      marginBottom: "5px",
      fontWeight: "600",
    },
    fieldValue: {
      fontSize: "16px",
      color: "white",
      backgroundColor: "rgba(255,255,255,0.05)",
      padding: "12px",
      borderRadius: "8px",
      border: "1px solid #2C3E50",
      marginBottom: "18px",
    },
    input: {
      width: "100%",
      padding: "12px",
      backgroundColor: "#0f172a",
      border: "1px solid #334155",
      borderRadius: "8px",
      color: "white",
      fontSize: "15px",
      marginBottom: "18px",
      outline: "none",
    },
    btnPrimary: {
      width: "100%",
      backgroundColor: "#2ECC71",
      color: "#1E2A38",
      border: "none",
      padding: "12px",
      borderRadius: "8px",
      fontWeight: "bold",
      cursor: "pointer",
      fontSize: "16px",
      marginTop: "10px",
    },
    btnSecondary: {
      background: "transparent",
      color: "#94a3b8",
      border: "1px solid #475569",
      padding: "10px 20px",
      borderRadius: "8px",
      cursor: "pointer",
      marginTop: "10px",
    },
    alert: {
      backgroundColor: "rgba(46, 204, 113, 0.2)",
      color: "#2ECC71",
      padding: "12px",
      borderRadius: "8px",
      textAlign: "center",
      border: "1px solid #2ECC71",
      marginBottom: "20px",
    },
    h3: {
      color: "#2ECC71",
      marginBottom: "20px",
      borderBottom: "2px solid #2ECC71",
      display: "inline-block",
      paddingBottom: "5px",
    },
  };

  if (loading)
    return (
      <div style={{ ...styles.wrapper, color: "white" }}>
        Loading Profile...
      </div>
    );
  if (!user) return null;

  return (
    <div style={styles.wrapper}>
      <div style={styles.card}>
        {/* Header */}
        <div style={styles.header}>
          <div style={styles.avatar}>
            {user.name ? user.name.charAt(0).toUpperCase() : "U"}
          </div>
          <div>
            <h2 style={styles.h2}>{user.name || "User"}</h2>
            <p style={styles.email}>{user.email}</p>
            <span style={styles.badge}>{user.proficiency || "Beginner"}</span>
          </div>
        </div>

        {message && <div style={styles.alert}>{message}</div>}
        <hr style={styles.divider} />

        {/* View Mode */}
        {!isEditing ? (
          <div>
            <h3 style={styles.h3}>Learning Profile</h3>

            <div>
              <label style={styles.label}>Technical Background</label>
              <div style={styles.fieldValue}>
                {user.technical_background || "Not set"}
              </div>
            </div>

            <div>
              <label style={styles.label}>Hardware Access</label>
              <div style={styles.fieldValue}>
                {user.hardware_access || "Not set"}
              </div>
            </div>

            <div>
              <label style={styles.label}>Learning Goals</label>
              <div style={styles.fieldValue}>
                {user.learning_goals || "Not set"}
              </div>
            </div>

            <button
              style={styles.btnPrimary}
              onClick={() => setIsEditing(true)}
            >
              Edit Profile
            </button>
          </div>
        ) : (
          /* Edit Mode */
          <form onSubmit={handleSubmit}>
            <h3 style={styles.h3}>Edit Profile</h3>

            <div>
              <label style={styles.label}>Technical Background</label>
              <input
                name="technical_background"
                value={profileData.technical_background}
                onChange={handleInputChange}
                placeholder="e.g. CS Student, Hobbyist..."
                style={styles.input}
              />
            </div>

            <div>
              <label style={styles.label}>Hardware Access</label>
              <input
                name="hardware_access"
                value={profileData.hardware_access}
                onChange={handleInputChange}
                placeholder="e.g. Arduino, Raspberry Pi, None"
                style={styles.input}
              />
            </div>

            <div>
              <label style={styles.label}>Learning Goals</label>
              <input
                name="learning_goals"
                value={profileData.learning_goals}
                onChange={handleInputChange}
                placeholder="e.g. Build a walking robot"
                style={styles.input}
              />
            </div>

            <div style={{ display: "flex", gap: "10px" }}>
              <button type="submit" style={styles.btnPrimary}>
                Save Changes
              </button>
              <button
                type="button"
                style={styles.btnSecondary}
                onClick={() => setIsEditing(false)}
              >
                Cancel
              </button>
            </div>
          </form>
        )}
      </div>
    </div>
  );
};

const Profile = () => {
  return (
    <Layout title="My Profile">
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => <ProfileContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default Profile;
