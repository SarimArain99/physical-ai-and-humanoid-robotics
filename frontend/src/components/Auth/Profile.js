import React, { useState, useEffect } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../components/Auth/AuthProvider"; // Check path!
import { useHistory } from "@docusaurus/router";

const Profile = () => {
  const { user, updateProfile, loading } = useAuth();
  const history = useHistory();

  const [isEditing, setIsEditing] = useState(false);
  const [profileData, setProfileData] = useState({
    technical_background: "",
    hardware_access: "",
    learning_goals: "",
  });
  const [message, setMessage] = useState("");

  // Redirect if not logged in
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
    setProfileData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setMessage("");

    const result = await updateProfile(profileData);
    if (result.success) {
      setMessage("Profile updated successfully!");
      setIsEditing(false);
      setTimeout(() => setMessage(""), 3000);
    } else {
      setMessage(result.message || "Failed to update profile");
    }
  };

  if (loading) {
    return (
      <Layout title="Profile">
        <div className="profile-container" style={{ textAlign: "center" }}>
          <h2>Loading Profile...</h2>
        </div>
      </Layout>
    );
  }

  if (!user) return null;

  // Helper to get initials
  const getInitials = (name) => {
    return name ? name.charAt(0).toUpperCase() : "U";
  };

  return (
    <Layout title="My Profile">
      <div className="profile-container">
        <h2>User Profile</h2>

        {message && (
          <div
            className={`profile-message ${
              message.includes("successfully") ? "success" : "error"
            }`}
          >
            {message}
          </div>
        )}

        {!isEditing ? (
          /* --- VIEW MODE --- */
          <div className="profile-view">
            <div className="profile-card-header">
              <div className="profile-avatar">
                {getInitials(user.name || user.email)}
              </div>
              <div className="profile-info">
                <h3>{user.name || "Student"}</h3>
                <p>{user.email}</p>
              </div>
            </div>

            <div className="profile-details-grid">
              <div>
                <span className="profile-field-label">
                  Technical Background
                </span>
                <div className="profile-field-content">
                  {profileData.technical_background ||
                    "No background specified yet."}
                </div>
              </div>

              <div>
                <span className="profile-field-label">Hardware Access</span>
                <div className="profile-field-content">
                  {profileData.hardware_access || "No hardware details added."}
                </div>
              </div>

              <div>
                <span className="profile-field-label">Learning Goals</span>
                <div className="profile-field-content">
                  {profileData.learning_goals || "No goals set yet."}
                </div>
              </div>
            </div>

            <div className="profile-actions">
              <button
                className="btn-primary"
                onClick={() => setIsEditing(true)}
              >
                Edit Profile
              </button>
            </div>
          </div>
        ) : (
          /* --- EDIT MODE --- */
          <form className="profile-form" onSubmit={handleSubmit}>
            <div className="profile-input-group">
              <label htmlFor="technical_background">Technical Background</label>
              <textarea
                id="technical_background"
                name="technical_background"
                value={profileData.technical_background}
                onChange={handleInputChange}
                placeholder="E.g., I know Python and basic C++, but new to ROS."
              />
            </div>

            <div className="profile-input-group">
              <label htmlFor="hardware_access">Hardware Access</label>
              <textarea
                id="hardware_access"
                name="hardware_access"
                value={profileData.hardware_access}
                onChange={handleInputChange}
                placeholder="E.g., I have a Raspberry Pi 4 and a servo motor kit."
              />
            </div>

            <div className="profile-input-group">
              <label htmlFor="learning_goals">Learning Goals</label>
              <textarea
                id="learning_goals"
                name="learning_goals"
                value={profileData.learning_goals}
                onChange={handleInputChange}
                placeholder="E.g., I want to build a robot arm that can pick up a cup."
              />
            </div>

            <div className="profile-actions">
              <button type="submit" className="btn-primary">
                Save Changes
              </button>
              <button
                type="button"
                className="btn-secondary"
                onClick={() => {
                  setIsEditing(false);
                  // Reset logic can go here if needed
                }}
              >
                Cancel
              </button>
            </div>
          </form>
        )}
      </div>
    </Layout>
  );
};

export default Profile;
