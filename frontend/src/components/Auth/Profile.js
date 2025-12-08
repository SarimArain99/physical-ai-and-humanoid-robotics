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
      setIsEditing(false);
    } else {
      setMessage(result.message || "Failed to update");
    }
  };

  if (loading) return <div className="profile-container">Loading...</div>;
  if (!user) return null;

  return (
    <div className="profile-wrapper">
      <div className="profile-card">
        <h2>My Profile</h2>

        <div className="profile-info">
          <p>
            <strong>Name:</strong> {user.name}
          </p>
          <p>
            <strong>Email:</strong> {user.email}
          </p>
          <p>
            <strong>Level:</strong>{" "}
            <span className="tag">{user.proficiency || "Beginner"}</span>
          </p>
        </div>

        {message && <div className="profile-msg">{message}</div>}

        {!isEditing ? (
          <div className="profile-details">
            <h3>Learning Preferences</h3>
            <p>
              <strong>Background:</strong>{" "}
              {profileData.technical_background || "Not set"}
            </p>
            <p>
              <strong>Hardware:</strong>{" "}
              {profileData.hardware_access || "Not set"}
            </p>
            <p>
              <strong>Goals:</strong> {profileData.learning_goals || "Not set"}
            </p>
            <button className="btn-primary" onClick={() => setIsEditing(true)}>
              Edit Profile
            </button>
          </div>
        ) : (
          <form onSubmit={handleSubmit} className="profile-form">
            <div className="form-group">
              <label>Technical Background</label>
              <input
                name="technical_background"
                value={profileData.technical_background}
                onChange={handleInputChange}
                placeholder="e.g. CS Student, Hobbyist..."
              />
            </div>
            <div className="form-group">
              <label>Hardware Access</label>
              <input
                name="hardware_access"
                value={profileData.hardware_access}
                onChange={handleInputChange}
                placeholder="e.g. Arduino, Raspberry Pi, None"
              />
            </div>
            <div className="form-group">
              <label>Learning Goals</label>
              <input
                name="learning_goals"
                value={profileData.learning_goals}
                onChange={handleInputChange}
                placeholder="e.g. Build a walking robot"
              />
            </div>
            <div className="btn-group">
              <button type="submit" className="btn-primary">
                Save Changes
              </button>
              <button
                type="button"
                className="btn-secondary"
                onClick={() => setIsEditing(false)}
              >
                Cancel
              </button>
            </div>
          </form>
        )}
      </div>

      {/* 🟢 EMBEDDED STYLES */}
      <style>{`
        .profile-wrapper { display: flex; justify-content: center; padding: 4rem 1rem; background-color: #151e29; min-height: 80vh; }
        .profile-card { background-color: #1E2A38; padding: 2.5rem; border-radius: 12px; width: 100%; max-width: 600px; box-shadow: 0 10px 30px rgba(0,0,0,0.3); border: 1px solid #2C3E50; color: white; }
        h2 { border-bottom: 2px solid #2ECC71; padding-bottom: 10px; margin-bottom: 20px; color: white; }
        h3 { color: #2ECC71; margin-top: 20px; }
        .profile-info p { margin: 8px 0; color: #cbd5e1; }
        .tag { background-color: #8b5cf6; padding: 2px 8px; border-radius: 12px; font-size: 12px; color: white; }
        .profile-msg { background: rgba(46, 204, 113, 0.2); color: #2ECC71; padding: 10px; border-radius: 6px; margin: 15px 0; text-align: center; }
        
        .form-group { margin-bottom: 15px; }
        .form-group label { display: block; margin-bottom: 5px; color: #94a3b8; font-size: 14px; }
        .form-group input { width: 100%; padding: 10px; background: #0f172a; border: 1px solid #334155; border-radius: 6px; color: white; }
        .form-group input:focus { border-color: #2ECC71; outline: none; }
        
        .btn-group { display: flex; gap: 10px; margin-top: 20px; }
        .btn-primary { background-color: #2ECC71; color: #1E2A38; border: none; padding: 10px 20px; border-radius: 6px; cursor: pointer; font-weight: bold; }
        .btn-primary:hover { background-color: #22c55e; }
        .btn-secondary { background-color: transparent; color: #cbd5e1; border: 1px solid #475569; padding: 10px 20px; border-radius: 6px; cursor: pointer; }
        .btn-secondary:hover { border-color: white; color: white; }
      `}</style>
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
