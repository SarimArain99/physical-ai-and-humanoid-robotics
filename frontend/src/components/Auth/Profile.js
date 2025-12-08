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

  if (loading)
    return (
      <div style={{ padding: "50px", textAlign: "center", color: "white" }}>
        Loading...
      </div>
    );
  if (!user) return null;

  return (
    <div id="force-profile-page">
      <div id="force-profile-card">
        {/* Header */}
        <div className="profile-header">
          <div className="profile-avatar">
            {user.name ? user.name.charAt(0).toUpperCase() : "U"}
          </div>
          <div>
            <h2>{user.name || "User"}</h2>
            <p className="email-text">{user.email}</p>
            <span className="badge">{user.proficiency || "Beginner"}</span>
          </div>
        </div>

        {message && <div className="alert-box">{message}</div>}
        <hr className="divider" />

        {/* View Mode */}
        {!isEditing ? (
          <div className="view-mode">
            <h3>Learning Profile</h3>

            <div className="field-group">
              <label>Technical Background</label>
              <div className="value-box">
                {user.technical_background || "Not set"}
              </div>
            </div>

            <div className="field-group">
              <label>Hardware Access</label>
              <div className="value-box">
                {user.hardware_access || "Not set"}
              </div>
            </div>

            <div className="field-group">
              <label>Learning Goals</label>
              <div className="value-box">
                {user.learning_goals || "Not set"}
              </div>
            </div>

            <button className="btn-primary" onClick={() => setIsEditing(true)}>
              Edit Profile
            </button>
          </div>
        ) : (
          /* Edit Mode */
          <form onSubmit={handleSubmit} className="edit-mode">
            <h3>Edit Profile</h3>

            <div className="field-group">
              <label>Technical Background</label>
              <input
                name="technical_background"
                value={profileData.technical_background}
                onChange={handleInputChange}
                placeholder="e.g. CS Student..."
              />
            </div>

            <div className="field-group">
              <label>Hardware Access</label>
              <input
                name="hardware_access"
                value={profileData.hardware_access}
                onChange={handleInputChange}
                placeholder="e.g. Arduino..."
              />
            </div>

            <div className="field-group">
              <label>Learning Goals</label>
              <input
                name="learning_goals"
                value={profileData.learning_goals}
                onChange={handleInputChange}
                placeholder="e.g. Robotics..."
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

      {/* 🛑 STRONG CSS OVERRIDE 
         Using ID selectors (#) forces these styles to apply over anything else.
      */}
      <style>{`
        #force-profile-page {
          display: flex;
          justify-content: center;
          padding: 4rem 1rem;
          background-color: #151e29 !important;
          min-height: 85vh;
        }

        #force-profile-card {
          background-color: #1E2A38 !important;
          padding: 2.5rem;
          border-radius: 16px;
          width: 100%;
          max-width: 600px;
          box-shadow: 0 10px 40px rgba(0,0,0,0.5);
          border: 1px solid #2C3E50;
          color: white !important;
        }

        #force-profile-card h2 { color: white !important; margin: 0; font-size: 24px; }
        #force-profile-card h3 { color: #2ECC71 !important; margin-bottom: 20px; border-bottom: 2px solid #2ECC71; display: inline-block; }
        
        .profile-header { display: flex; align-items: center; gap: 20px; margin-bottom: 20px; }
        
        .profile-avatar {
          width: 70px; height: 70px;
          background: linear-gradient(135deg, #2ECC71, #27ae60);
          border-radius: 50%;
          display: flex; align-items: center; justify-content: center;
          font-size: 32px; font-weight: bold; color: #1E2A38;
        }

        .email-text { color: #94a3b8 !important; margin: 5px 0 10px 0; font-size: 14px; }
        
        .badge {
          background-color: #8b5cf6;
          color: white;
          padding: 4px 10px;
          border-radius: 12px;
          font-size: 11px;
          text-transform: uppercase;
          font-weight: bold;
        }

        .divider { border: 0; height: 1px; background: #2C3E50; margin: 20px 0; }

        .field-group { margin-bottom: 18px; }
        .field-group label {
          display: block; color: #94a3b8; font-size: 12px; 
          text-transform: uppercase; margin-bottom: 5px; font-weight: 600;
        }

        .value-box {
          font-size: 16px; color: white;
          background: rgba(255,255,255,0.05);
          padding: 12px; border-radius: 8px;
          border: 1px solid #2C3E50;
        }

        #force-profile-card input {
          width: 100%; padding: 12px;
          background-color: #0f172a !important;
          border: 1px solid #334155 !important;
          border-radius: 8px;
          color: white !important;
          font-size: 15px;
          outline: none;
        }
        #force-profile-card input:focus { border-color: #2ECC71 !important; }

        .btn-primary {
          width: 100%; background-color: #2ECC71; color: #1E2A38;
          border: none; padding: 12px; border-radius: 8px;
          font-weight: bold; cursor: pointer; font-size: 16px; margin-top: 10px;
        }
        .btn-primary:hover { background-color: #22c55e; }

        .btn-secondary {
          background: transparent; color: #94a3b8;
          border: 1px solid #475569; padding: 10px 20px;
          border-radius: 8px; cursor: pointer;
        }
        .btn-secondary:hover { color: white; border-color: white; }
        .btn-group { display: flex; gap: 10px; margin-top: 10px; }

        .alert-box {
          background: rgba(46, 204, 113, 0.2);
          color: #2ECC71; padding: 12px;
          border-radius: 8px; text-align: center;
          border: 1px solid #2ECC71; margin-bottom: 20px;
        }
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
